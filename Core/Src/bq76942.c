#include "bq76942.h"

#include <string.h>

#include "main.h"
#include "lithium_config.h"


extern SPI_HandleTypeDef hspi1;


/* ============================================================
 * SPI PROTOCOL
 * ============================================================ */

#define BQ76942_SPI_TIMEOUT_MS                     5U
#define BQ76942_SPI_RESPONSE_TIMEOUT_MS            20U
#define BQ76942_SUBCOMMAND_TIMEOUT_MS              50U

#define BQ76942_SPI_READ_BIT                       0x00U
#define BQ76942_SPI_WRITE_BIT                      0x80U
#define BQ76942_SPI_ADDRESS_MASK                   0x7FU
#define BQ76942_SPI_READ_DUMMY                     0xFFU

#define BQ76942_SPI_CRC_POLYNOMIAL                 0x07U

#define BQ76942_SPI_MAX_RETRIES                    8U


/* ============================================================
 * DIRECT COMMANDS
 * ============================================================ */

#define BQ76942_CMD_SAFETY_STATUS_A                0x03U
#define BQ76942_CMD_SAFETY_STATUS_B                0x05U
#define BQ76942_CMD_SAFETY_STATUS_C                0x07U

#define BQ76942_CMD_CELL1_VOLTAGE                  0x14U
#define BQ76942_CMD_CELL2_VOLTAGE                  0x16U
#define BQ76942_CMD_CELL10_VOLTAGE                 0x26U

#define BQ76942_CMD_ALARM_STATUS                   0x62U
#define BQ76942_CMD_ALARM_ENABLE                   0x66U

#define BQ76942_CMD_INTERNAL_TEMPERATURE           0x68U
#define BQ76942_CMD_TS1_TEMPERATURE                0x70U
#define BQ76942_CMD_TS2_TEMPERATURE                0x72U
#define BQ76942_CMD_TS3_TEMPERATURE                0x74U


/* ============================================================
 * SUBCOMMAND / DATA MEMORY BUFFER
 * ============================================================ */

#define BQ76942_CMD_SUBCOMMAND_LOW                 0x3EU
#define BQ76942_CMD_SUBCOMMAND_HIGH                0x3FU

#define BQ76942_CMD_TRANSFER_BUFFER                0x40U

#define BQ76942_CMD_CHECKSUM                       0x60U
#define BQ76942_CMD_LENGTH                         0x61U


/* ============================================================
 * SUBCOMMANDS
 * ============================================================ */

#define BQ76942_SUBCMD_DEVICE_NUMBER               0x0001U

#define BQ76942_SUBCMD_CB_ACTIVE_CELLS             0x0083U

#define BQ76942_SUBCMD_SET_CFGUPDATE               0x0090U
#define BQ76942_SUBCMD_EXIT_CFGUPDATE              0x0092U

#define BQ76942_SUBCMD_SLEEP_DISABLE               0x009AU

#define BQ76942_EXPECTED_DEVICE_NUMBER             0x7694U


/* ============================================================
 * DATA MEMORY
 * ============================================================ */

#define BQ76942_RAM_ENABLED_PROTECTIONS_A          0x9261U
#define BQ76942_RAM_ENABLED_PROTECTIONS_B          0x9262U
#define BQ76942_RAM_ENABLED_PROTECTIONS_C          0x9263U

#define BQ76942_RAM_DEFAULT_ALARM_MASK             0x926DU

#define BQ76942_RAM_CUV_THRESHOLD                  0x9275U
#define BQ76942_RAM_CUV_DELAY                      0x9276U
#define BQ76942_RAM_COV_THRESHOLD                  0x9278U
#define BQ76942_RAM_COV_DELAY                      0x9279U
#define BQ76942_RAM_CUV_RECOVERY_HYST              0x927BU
#define BQ76942_RAM_COV_RECOVERY_HYST              0x927CU

#define BQ76942_RAM_ALERT_PIN_CONFIG               0x92FCU
#define BQ76942_RAM_TS1_CONFIG                     0x92FDU
#define BQ76942_RAM_TS2_CONFIG                     0x92FEU
#define BQ76942_RAM_TS3_CONFIG                     0x92FFU

#define BQ76942_RAM_VCELL_MODE                     0x9304U

#define BQ76942_RAM_BALANCING_CONFIG               0x9335U
#define BQ76942_RAM_BALANCE_MIN_TEMP               0x9336U
#define BQ76942_RAM_BALANCE_MAX_TEMP               0x9337U
#define BQ76942_RAM_BALANCE_MAX_INTERNAL_TEMP      0x9338U
#define BQ76942_RAM_BALANCE_INTERVAL               0x9339U
#define BQ76942_RAM_BALANCE_MAX_CELLS              0x933AU

#define BQ76942_RAM_BALANCE_MIN_CELL_CHARGE        0x933BU
#define BQ76942_RAM_BALANCE_START_DELTA_CHARGE     0x933DU
#define BQ76942_RAM_BALANCE_STOP_DELTA_CHARGE      0x933EU

#define BQ76942_RAM_BALANCE_MIN_CELL_RELAX         0x933FU
#define BQ76942_RAM_BALANCE_START_DELTA_RELAX      0x9341U
#define BQ76942_RAM_BALANCE_STOP_DELTA_RELAX       0x9342U


/* ============================================================
 * BOARD CONFIGURATION
 * ============================================================ */

#define BQ76942_VCELL_MODE_3S                      0x0203U

#define BQ76942_TS_CONFIG_103AT                    0x07U

#define BQ76942_ALERT_CONFIG_ACTIVE_LOW            0x82U

#define BQ76942_ENABLED_PROTECTIONS_A              0x0CU
#define BQ76942_ENABLED_PROTECTIONS_B              0x00U
#define BQ76942_ENABLED_PROTECTIONS_C              0x00U

#define BQ76942_DEFAULT_ALARM_MASK                 0xF800U


/*
 * 49 * 50.6 mV ~= 2.48 V
 * 83 * 50.6 mV ~= 4.20 V
 */
/* 50 x 50.6 mV = approximately 2.53 V. */
#define BQ76942_CUV_THRESHOLD_CODE                 50U
#define BQ76942_COV_THRESHOLD_CODE                 83U


/*
 * delay ~= 6.6 ms + code * 3.3 ms
 */
#define BQ76942_CUV_DELAY_CODE                     604U
#define BQ76942_COV_DELAY_CODE                     150U


/*
 * 2 * 50.6 mV ~= 101 mV.
 */
#define BQ76942_CUV_RECOVERY_HYST_CODE             2U
#define BQ76942_COV_RECOVERY_HYST_CODE             2U


#define BQ76942_BALANCING_HOST_ONLY                0x00U


/* ============================================================
 * SAFETY STATUS
 * ============================================================ */

#define BQ76942_SAFETY_A_CUV_MASK                  (1U << 2)
#define BQ76942_SAFETY_A_COV_MASK                  (1U << 3)

#define BQ76942_SAFETY_B_UTC_MASK                  (1U << 0)
#define BQ76942_SAFETY_B_OTC_MASK                  (1U << 4)
#define BQ76942_SAFETY_B_OTD_MASK                  (1U << 5)
#define BQ76942_SAFETY_B_OTINT_MASK                (1U << 6)


/* ============================================================
 * PERIODIC READ STATE
 * ============================================================ */

typedef enum
{
    BQ76942_READ_IDLE = 0,

    BQ76942_READ_CELL1,
    BQ76942_READ_CELL2,
    BQ76942_READ_CELL3,

    BQ76942_READ_TS1,
    BQ76942_READ_TS2,
    BQ76942_READ_TS3,

    BQ76942_READ_INTERNAL_TEMP,

    BQ76942_READ_SAFETY_A,
    BQ76942_READ_SAFETY_B,
    BQ76942_READ_SAFETY_C,

    BQ76942_READ_ALARM,

    BQ76942_READ_COMPLETE,
    BQ76942_READ_ERROR

} bq76942_read_state_t;


typedef enum
{
    BQ76942_ASYNC_IDLE = 0,
    BQ76942_ASYNC_WAIT_LOW_BYTE,
    BQ76942_ASYNC_WAIT_HIGH_BYTE

} bq76942_async_state_t;


/* ============================================================
 * PRIVATE STATE
 * ============================================================ */

static bq76942_read_state_t read_state =
        BQ76942_READ_IDLE;

static bq76942_async_state_t async_state =
        BQ76942_ASYNC_IDLE;


static bq76942_data_t working_data;
static bq76942_data_t completed_data;


static uint8_t spi_tx[3];
static uint8_t spi_rx[3];

static volatile bool spi_transfer_complete = false;
static volatile bool spi_transfer_error = false;

static bool data_ready = false;


static uint8_t async_base_address = 0U;
static uint8_t async_length = 0U;
static uint8_t async_low_byte = 0U;
static uint8_t async_retry_count = 0U;

static uint32_t async_transaction_start_ms = 0U;


/* ============================================================
 * PRIVATE PROTOTYPES
 * ============================================================ */

static uint8_t BQ76942_CalculateCRC(
        const uint8_t *data,
        uint8_t length);

static bool BQ76942_ResponseValid(
        uint8_t expected_command,
        const uint8_t response[3]);

static bq_status_t BQ76942_TransferBlocking(
        uint8_t command,
        uint8_t data,
        uint8_t response[3]);

static bq_status_t BQ76942_ReadByteBlocking(
        uint8_t address,
        uint8_t *value);

static bq_status_t BQ76942_ReadWordBlocking(
        uint8_t address,
        uint16_t *value);

static bq_status_t BQ76942_WriteByteBlocking(
        uint8_t address,
        uint8_t value);

static bq_status_t BQ76942_WriteWordBlocking(
        uint8_t address,
        uint16_t value);

static bq_status_t BQ76942_SendSubcommandBlocking(
        uint16_t subcommand);

static bq_status_t BQ76942_WaitSubcommandComplete(
        uint16_t subcommand);

static bq_status_t BQ76942_ReadSubcommandWordBlocking(
        uint16_t subcommand,
        uint16_t *value);

static bq_status_t BQ76942_WriteSubcommandWordBlocking(
        uint16_t subcommand,
        uint16_t value);

static bq_status_t BQ76942_WriteDataMemory(
        uint16_t address,
        const uint8_t *data,
        uint8_t length);

static bq_status_t BQ76942_WriteDataMemoryU8(
        uint16_t address,
        uint8_t value);

static bq_status_t BQ76942_WriteDataMemoryU16(
        uint16_t address,
        uint16_t value);

static uint8_t BQ76942_BlockChecksum(
        const uint8_t *data,
        uint8_t length);

static bool BQ76942_StartAsyncReadByte(
        uint8_t address);

static bool BQ76942_StartAsyncDirectRead(
        uint8_t address,
        uint8_t length);

static bool BQ76942_ServiceAsyncDirectRead(
        uint16_t *value);

static bool BQ76942_AsyncTimedOut(void);


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

bq_status_t BQ76942_Init(void)
{
    bq_status_t status;

    uint16_t device_number;


    HAL_GPIO_WritePin(
            BQ2_CS_GPIO_Port,
            BQ2_CS_Pin,
            GPIO_PIN_SET);


    status =
            BQ76942_ReadSubcommandWordBlocking(
                    BQ76942_SUBCMD_DEVICE_NUMBER,
                    &device_number);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if (device_number !=
        BQ76942_EXPECTED_DEVICE_NUMBER)
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    status =
            BQ76942_SendSubcommandBlocking(
                    BQ76942_SUBCMD_SLEEP_DISABLE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_SendSubcommandBlocking(
                    BQ76942_SUBCMD_SET_CFGUPDATE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    /*
     * Startup-only bounded CONFIG_UPDATE settling time.
     *
     * No HAL_Delay() is used in normal runtime monitoring.
     */
    {
        uint32_t start_ms =
                HAL_GetTick();

        while ((HAL_GetTick() -
                start_ms) < 2U)
        {
        }
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_ALERT_PIN_CONFIG,
                    BQ76942_ALERT_CONFIG_ACTIVE_LOW);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_TS1_CONFIG,
                    BQ76942_TS_CONFIG_103AT);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_TS2_CONFIG,
                    BQ76942_TS_CONFIG_103AT);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_TS3_CONFIG,
                    BQ76942_TS_CONFIG_103AT);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_VCELL_MODE,
                    BQ76942_VCELL_MODE_3S);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_ENABLED_PROTECTIONS_A,
                    BQ76942_ENABLED_PROTECTIONS_A);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_ENABLED_PROTECTIONS_B,
                    BQ76942_ENABLED_PROTECTIONS_B);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_ENABLED_PROTECTIONS_C,
                    BQ76942_ENABLED_PROTECTIONS_C);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_CUV_THRESHOLD,
                    BQ76942_CUV_THRESHOLD_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_CUV_DELAY,
                    BQ76942_CUV_DELAY_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_CUV_RECOVERY_HYST,
                    BQ76942_CUV_RECOVERY_HYST_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_COV_THRESHOLD,
                    BQ76942_COV_THRESHOLD_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_COV_DELAY,
                    BQ76942_COV_DELAY_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_COV_RECOVERY_HYST,
                    BQ76942_COV_RECOVERY_HYST_CODE);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_DEFAULT_ALARM_MASK,
                    BQ76942_DEFAULT_ALARM_MASK);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCING_CONFIG,
                    BQ76942_BALANCING_HOST_ONLY);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_MIN_TEMP,
                    0U);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_MAX_TEMP,
                    45U);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_MAX_INTERNAL_TEMP,
                    70U);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_INTERVAL,
                    LITHIUM_BALANCE_IC_INTERVAL_S);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_MAX_CELLS,
                    LITHIUM_BALANCE_MAX_SIMULTANEOUS_CELLS);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_BALANCE_MIN_CELL_CHARGE,
                    LITHIUM_BALANCE_MIN_CELL_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_START_DELTA_CHARGE,
                    LITHIUM_BALANCE_START_DELTA_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_STOP_DELTA_CHARGE,
                    LITHIUM_BALANCE_STOP_DELTA_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU16(
                    BQ76942_RAM_BALANCE_MIN_CELL_RELAX,
                    LITHIUM_BALANCE_MIN_CELL_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_START_DELTA_RELAX,
                    LITHIUM_BALANCE_START_DELTA_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_WriteDataMemoryU8(
                    BQ76942_RAM_BALANCE_STOP_DELTA_RELAX,
                    LITHIUM_BALANCE_STOP_DELTA_MV);

    if (status != BQ_STATUS_OK)
    {
        goto configuration_failed;
    }


    status =
            BQ76942_SendSubcommandBlocking(
                    BQ76942_SUBCMD_EXIT_CFGUPDATE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    {
        uint32_t start_ms =
                HAL_GetTick();

        while ((HAL_GetTick() -
                start_ms) < 2U)
        {
        }
    }


    status =
            BQ76942_WriteWordBlocking(
                    BQ76942_CMD_ALARM_ENABLE,
                    BQ76942_DEFAULT_ALARM_MASK);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_SetBalancingMask(
                    0U);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    read_state =
            BQ76942_READ_IDLE;

    async_state =
            BQ76942_ASYNC_IDLE;

    data_ready =
            false;

    spi_transfer_complete =
            false;

    spi_transfer_error =
            false;

    async_transaction_start_ms =
            0U;


    return BQ_STATUS_OK;


configuration_failed:

    (void)
    BQ76942_SendSubcommandBlocking(
            BQ76942_SUBCMD_EXIT_CFGUPDATE);


    return BQ_STATUS_CONFIG_ERROR;
}


/* ============================================================
 * START SNAPSHOT
 * ============================================================ */

bq_status_t BQ76942_StartReadData(void)
{
    if (read_state !=
        BQ76942_READ_IDLE)
    {
        return BQ_STATUS_BUSY;
    }


    memset(
            &working_data,
            0,
            sizeof(working_data));


    data_ready =
            false;


    async_state =
            BQ76942_ASYNC_IDLE;


    read_state =
            BQ76942_READ_CELL1;


    return BQ_STATUS_OK;
}


/* ============================================================
 * NON-BLOCKING SERVICE
 * ============================================================ */

void BQ76942_Service(void)
{
    uint16_t value;


    switch (read_state)
    {
        case BQ76942_READ_IDLE:

            break;


        case BQ76942_READ_CELL1:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_CELL1_VOLTAGE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.cell_mv[0] =
                    value;


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_CELL2;

            break;


        case BQ76942_READ_CELL2:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_CELL2_VOLTAGE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.cell_mv[1] =
                    value;


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_CELL3;

            break;


        case BQ76942_READ_CELL3:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_CELL10_VOLTAGE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.cell_mv[2] =
                    value;


            working_data.pack_mv =
                    (uint16_t)(
                            working_data.cell_mv[0] +
                            working_data.cell_mv[1] +
                            working_data.cell_mv[2]);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_TS1;

            break;


        case BQ76942_READ_TS1:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_TS1_TEMPERATURE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.temperature_dC[0] =
                    (int16_t)((int16_t)value - 2732);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_TS2;

            break;


        case BQ76942_READ_TS2:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_TS2_TEMPERATURE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.temperature_dC[1] =
                    (int16_t)((int16_t)value - 2732);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_TS3;

            break;


        case BQ76942_READ_TS3:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_TS3_TEMPERATURE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.temperature_dC[2] =
                    (int16_t)((int16_t)value - 2732);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_INTERNAL_TEMP;

            break;


        case BQ76942_READ_INTERNAL_TEMP:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_INTERNAL_TEMPERATURE,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.internal_temperature_dC =
                    (int16_t)((int16_t)value - 2732);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_SAFETY_A;

            break;


        case BQ76942_READ_SAFETY_A:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_SAFETY_STATUS_A,
                        1U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.safety_status_a =
                    (uint8_t)value;


            working_data.cell_undervoltage =
                    ((working_data.safety_status_a &
                      BQ76942_SAFETY_A_CUV_MASK) != 0U);


            working_data.cell_overvoltage =
                    ((working_data.safety_status_a &
                      BQ76942_SAFETY_A_COV_MASK) != 0U);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_SAFETY_B;

            break;


        case BQ76942_READ_SAFETY_B:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_SAFETY_STATUS_B,
                        1U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.safety_status_b =
                    (uint8_t)value;


            working_data.charge_undertemperature =
                    ((working_data.safety_status_b &
                      BQ76942_SAFETY_B_UTC_MASK) != 0U);


            working_data.discharge_overtemperature =
                    ((working_data.safety_status_b &
                      BQ76942_SAFETY_B_OTD_MASK) != 0U);


            working_data.charge_overtemperature =
                    ((working_data.safety_status_b &
                      BQ76942_SAFETY_B_OTC_MASK) != 0U);


            working_data.internal_overtemperature =
                    ((working_data.safety_status_b &
                      BQ76942_SAFETY_B_OTINT_MASK) != 0U);


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_SAFETY_C;

            break;


        case BQ76942_READ_SAFETY_C:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_SAFETY_STATUS_C,
                        1U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.safety_status_c =
                    (uint8_t)value;


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_ALARM;

            break;


        case BQ76942_READ_ALARM:

            if (async_state ==
                BQ76942_ASYNC_IDLE)
            {
                if (!BQ76942_StartAsyncDirectRead(
                        BQ76942_CMD_ALARM_STATUS,
                        2U))
                {
                    read_state =
                            BQ76942_READ_ERROR;
                }

                break;
            }


            if (!BQ76942_ServiceAsyncDirectRead(
                    &value))
            {
                break;
            }


            working_data.alarm_status =
                    value;


            async_state =
                    BQ76942_ASYNC_IDLE;

            read_state =
                    BQ76942_READ_COMPLETE;

            break;


        case BQ76942_READ_COMPLETE:

            completed_data =
                    working_data;


            data_ready =
                    true;


            read_state =
                    BQ76942_READ_IDLE;

            async_state =
                    BQ76942_ASYNC_IDLE;

            break;


        case BQ76942_READ_ERROR:

        default:

            HAL_GPIO_WritePin(
                    BQ2_CS_GPIO_Port,
                    BQ2_CS_Pin,
                    GPIO_PIN_SET);


            spi_transfer_complete =
                    false;

            spi_transfer_error =
                    false;

            async_transaction_start_ms =
                    0U;

            async_state =
                    BQ76942_ASYNC_IDLE;

            data_ready =
                    false;

            read_state =
                    BQ76942_READ_IDLE;

            break;
    }
}


/* ============================================================
 * SNAPSHOT ACCESS
 * ============================================================ */

bool BQ76942_DataReady(void)
{
    return data_ready;
}


bq_status_t BQ76942_GetData(
        bq76942_data_t *data)
{
    if (data == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    if (!data_ready)
    {
        return BQ_STATUS_BUSY;
    }


    *data =
            completed_data;


    data_ready =
            false;


    return BQ_STATUS_OK;
}


bool BQ76942_IsBusy(void)
{
    return
            (read_state !=
             BQ76942_READ_IDLE);
}


/* ============================================================
 * CALLBACK FLAGS
 * ============================================================ */

void BQ76942_NotifySpiTransferComplete(void)
{
    spi_transfer_complete =
            true;
}


void BQ76942_NotifySpiTransferError(void)
{
    spi_transfer_error =
            true;
}


/* ============================================================
 * BALANCING
 * ============================================================ */

bq_status_t BQ76942_SetBalancingMask(
        uint16_t cell_mask)
{
    const uint16_t allowed_mask =
            BQ76942_BALANCE_GROUP1_MASK |
            BQ76942_BALANCE_GROUP2_MASK |
            BQ76942_BALANCE_GROUP3_MASK;


    if ((cell_mask &
         (uint16_t)~allowed_mask) != 0U)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    if ((cell_mask != 0U) &&
        ((cell_mask &
          (cell_mask - 1U)) != 0U))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    return
            BQ76942_WriteSubcommandWordBlocking(
                    BQ76942_SUBCMD_CB_ACTIVE_CELLS,
                    cell_mask);
}


/* ============================================================
 * ALARM CLEAR
 * ============================================================ */

bq_status_t BQ76942_ClearAlarmStatus(
        uint16_t alarm_mask)
{
    if (alarm_mask == 0U)
    {
        return BQ_STATUS_OK;
    }


    return
            BQ76942_WriteWordBlocking(
                    BQ76942_CMD_ALARM_STATUS,
                    alarm_mask);
}


/* ============================================================
 * CRC
 * ============================================================ */

static uint8_t BQ76942_CalculateCRC(
        const uint8_t *data,
        uint8_t length)
{
    uint8_t crc =
            0U;

    uint8_t i;
    uint8_t bit;


    for (i = 0U;
         i < length;
         i++)
    {
        crc ^=
                data[i];


        for (bit = 0U;
             bit < 8U;
             bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc =
                        (uint8_t)(
                                (crc << 1) ^
                                BQ76942_SPI_CRC_POLYNOMIAL);
            }
            else
            {
                crc <<= 1;
            }
        }
    }


    return crc;
}


/* ============================================================
 * RESPONSE VALIDATION
 * ============================================================ */

static bool BQ76942_ResponseValid(
        uint8_t expected_command,
        const uint8_t response[3])
{
    uint8_t crc_data[2];


    if (response == NULL)
    {
        return false;
    }


    if (response[0] !=
        expected_command)
    {
        return false;
    }


    crc_data[0] =
            response[0];

    crc_data[1] =
            response[1];


    return
            (BQ76942_CalculateCRC(
                    crc_data,
                    2U) ==
             response[2]);
}


/* ============================================================
 * BLOCKING TRANSFER
 * ============================================================ */

static bq_status_t BQ76942_TransferBlocking(
        uint8_t command,
        uint8_t data,
        uint8_t response[3])
{
    HAL_StatusTypeDef hal_status;

    uint8_t tx[3];


    tx[0] =
            command;

    tx[1] =
            data;

    tx[2] =
            BQ76942_CalculateCRC(
                    tx,
                    2U);


    HAL_GPIO_WritePin(
            BQ2_CS_GPIO_Port,
            BQ2_CS_Pin,
            GPIO_PIN_RESET);


    hal_status =
            HAL_SPI_TransmitReceive(
                    &hspi1,
                    tx,
                    response,
                    3U,
                    BQ76942_SPI_TIMEOUT_MS);


    HAL_GPIO_WritePin(
            BQ2_CS_GPIO_Port,
            BQ2_CS_Pin,
            GPIO_PIN_SET);


    if (hal_status == HAL_TIMEOUT)
    {
        return BQ_STATUS_TIMEOUT;
    }


    if (hal_status != HAL_OK)
    {
        return BQ_STATUS_COMM_ERROR;
    }


    return BQ_STATUS_OK;
}


/* ============================================================
 * BLOCKING READ
 * ============================================================ */

static bq_status_t BQ76942_ReadByteBlocking(
        uint8_t address,
        uint8_t *value)
{
    bq_status_t status;

    uint8_t response[3];
    uint8_t command;

    uint8_t retry;


    if (value == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    command =
            BQ76942_SPI_READ_BIT |
            (address &
             BQ76942_SPI_ADDRESS_MASK);


    for (retry = 0U;
         retry < BQ76942_SPI_MAX_RETRIES;
         retry++)
    {
        status =
                BQ76942_TransferBlocking(
                        command,
                        BQ76942_SPI_READ_DUMMY,
                        response);

        if (status != BQ_STATUS_OK)
        {
            return status;
        }


        if (BQ76942_ResponseValid(
                command,
                response))
        {
            *value =
                    response[1];


            return BQ_STATUS_OK;
        }
    }


    return BQ_STATUS_TIMEOUT;
}


static bq_status_t BQ76942_ReadWordBlocking(
        uint8_t address,
        uint16_t *value)
{
    bq_status_t status;

    uint8_t low;
    uint8_t high;


    if (value == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    status =
            BQ76942_ReadByteBlocking(
                    address,
                    &low);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_ReadByteBlocking(
                    (uint8_t)(address + 1U),
                    &high);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    *value =
            (uint16_t)low |
            ((uint16_t)high << 8);


    return BQ_STATUS_OK;
}


/* ============================================================
 * BLOCKING WRITE
 * ============================================================ */

static bq_status_t BQ76942_WriteByteBlocking(
        uint8_t address,
        uint8_t value)
{
    bq_status_t status;

    uint8_t response[3];
    uint8_t command;
    uint8_t expected_crc;

    uint8_t retry;


    command =
            BQ76942_SPI_WRITE_BIT |
            (address &
             BQ76942_SPI_ADDRESS_MASK);


    {
        uint8_t crc_data[2];

        crc_data[0] =
                command;

        crc_data[1] =
                value;


        expected_crc =
                BQ76942_CalculateCRC(
                        crc_data,
                        2U);
    }


    for (retry = 0U;
         retry < BQ76942_SPI_MAX_RETRIES;
         retry++)
    {
        status =
                BQ76942_TransferBlocking(
                        command,
                        value,
                        response);

        if (status != BQ_STATUS_OK)
        {
            return status;
        }


        if ((response[0] == command) &&
            (response[1] == value) &&
            (response[2] == expected_crc))
        {
            return BQ_STATUS_OK;
        }
    }


    return BQ_STATUS_TIMEOUT;
}


static bq_status_t BQ76942_WriteWordBlocking(
        uint8_t address,
        uint16_t value)
{
    bq_status_t status;


    status =
            BQ76942_WriteByteBlocking(
                    address,
                    (uint8_t)(value & 0xFFU));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    return
            BQ76942_WriteByteBlocking(
                    (uint8_t)(address + 1U),
                    (uint8_t)(value >> 8));
}


/* ============================================================
 * SUBCOMMAND
 * ============================================================ */

static bq_status_t BQ76942_SendSubcommandBlocking(
        uint16_t subcommand)
{
    bq_status_t status;


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_LOW,
                    (uint8_t)(subcommand & 0xFFU));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    return
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_HIGH,
                    (uint8_t)(subcommand >> 8));
}


static bq_status_t BQ76942_WaitSubcommandComplete(
        uint16_t subcommand)
{
    uint32_t start_ms;

    uint16_t returned_command;

    bq_status_t status;


    start_ms =
            HAL_GetTick();


    while ((HAL_GetTick() -
            start_ms) <
           BQ76942_SUBCOMMAND_TIMEOUT_MS)
    {
        status =
                BQ76942_ReadWordBlocking(
                        BQ76942_CMD_SUBCOMMAND_LOW,
                        &returned_command);


        if ((status == BQ_STATUS_OK) &&
            (returned_command == subcommand))
        {
            return BQ_STATUS_OK;
        }
    }


    return BQ_STATUS_TIMEOUT;
}


static bq_status_t BQ76942_ReadSubcommandWordBlocking(
        uint16_t subcommand,
        uint16_t *value)
{
    bq_status_t status;


    if (value == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    status =
            BQ76942_SendSubcommandBlocking(
                    subcommand);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_WaitSubcommandComplete(
                    subcommand);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    return
            BQ76942_ReadWordBlocking(
                    BQ76942_CMD_TRANSFER_BUFFER,
                    value);
}


static bq_status_t BQ76942_WriteSubcommandWordBlocking(
        uint16_t subcommand,
        uint16_t value)
{
    bq_status_t status;

    uint8_t block[4];

    uint8_t checksum;


    block[0] =
            (uint8_t)(subcommand & 0xFFU);

    block[1] =
            (uint8_t)(subcommand >> 8);

    block[2] =
            (uint8_t)(value & 0xFFU);

    block[3] =
            (uint8_t)(value >> 8);


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_LOW,
                    block[0]);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_HIGH,
                    block[1]);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_TRANSFER_BUFFER,
                    block[2]);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_WriteByteBlocking(
                    (uint8_t)(
                            BQ76942_CMD_TRANSFER_BUFFER +
                            1U),
                    block[3]);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    checksum =
            BQ76942_BlockChecksum(
                    block,
                    4U);


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_CHECKSUM,
                    checksum);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    return
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_LENGTH,
                    6U);
}


/* ============================================================
 * DATA MEMORY
 * ============================================================ */

static bq_status_t BQ76942_WriteDataMemory(
        uint16_t address,
        const uint8_t *data,
        uint8_t length)
{
    bq_status_t status;

    uint8_t checksum_block[34];

    uint8_t checksum;

    uint8_t i;


    if ((data == NULL) ||
        (length == 0U) ||
        (length > 32U))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_LOW,
                    (uint8_t)(address & 0xFFU));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_SUBCOMMAND_HIGH,
                    (uint8_t)(address >> 8));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    checksum_block[0] =
            (uint8_t)(address & 0xFFU);

    checksum_block[1] =
            (uint8_t)(address >> 8);


    for (i = 0U;
         i < length;
         i++)
    {
        status =
                BQ76942_WriteByteBlocking(
                        (uint8_t)(
                                BQ76942_CMD_TRANSFER_BUFFER +
                                i),
                        data[i]);

        if (status != BQ_STATUS_OK)
        {
            return status;
        }


        checksum_block[i + 2U] =
                data[i];
    }


    checksum =
            BQ76942_BlockChecksum(
                    checksum_block,
                    (uint8_t)(length + 2U));


    status =
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_CHECKSUM,
                    checksum);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    return
            BQ76942_WriteByteBlocking(
                    BQ76942_CMD_LENGTH,
                    (uint8_t)(length + 4U));
}


static bq_status_t BQ76942_WriteDataMemoryU8(
        uint16_t address,
        uint8_t value)
{
    return
            BQ76942_WriteDataMemory(
                    address,
                    &value,
                    1U);
}


static bq_status_t BQ76942_WriteDataMemoryU16(
        uint16_t address,
        uint16_t value)
{
    uint8_t data[2];


    data[0] =
            (uint8_t)(value & 0xFFU);

    data[1] =
            (uint8_t)(value >> 8);


    return
            BQ76942_WriteDataMemory(
                    address,
                    data,
                    2U);
}


/* ============================================================
 * CHECKSUM
 * ============================================================ */

static uint8_t BQ76942_BlockChecksum(
        const uint8_t *data,
        uint8_t length)
{
    uint16_t sum =
            0U;

    uint8_t i;


    for (i = 0U;
         i < length;
         i++)
    {
        sum +=
                data[i];
    }


    return
            (uint8_t)(
                    0xFFU -
                    (sum & 0xFFU));
}


/* ============================================================
 * ASYNC BYTE READ
 * ============================================================ */

static bool BQ76942_StartAsyncReadByte(
        uint8_t address)
{
    HAL_StatusTypeDef hal_status;

    uint8_t command;


    command =
            BQ76942_SPI_READ_BIT |
            (address &
             BQ76942_SPI_ADDRESS_MASK);


    spi_tx[0] =
            command;

    spi_tx[1] =
            BQ76942_SPI_READ_DUMMY;

    spi_tx[2] =
            BQ76942_CalculateCRC(
                    spi_tx,
                    2U);


    spi_transfer_complete =
            false;

    spi_transfer_error =
            false;


    HAL_GPIO_WritePin(
            BQ2_CS_GPIO_Port,
            BQ2_CS_Pin,
            GPIO_PIN_RESET);


    hal_status =
            HAL_SPI_TransmitReceive_IT(
                    &hspi1,
                    spi_tx,
                    spi_rx,
                    3U);


    if (hal_status != HAL_OK)
    {
        HAL_GPIO_WritePin(
                BQ2_CS_GPIO_Port,
                BQ2_CS_Pin,
                GPIO_PIN_SET);


        return false;
    }


    async_transaction_start_ms =
            HAL_GetTick();


    return true;
}


/* ============================================================
 * START ASYNC DIRECT READ
 * ============================================================ */

static bool BQ76942_StartAsyncDirectRead(
        uint8_t address,
        uint8_t length)
{
    if ((length == 0U) ||
        (length > 2U))
    {
        return false;
    }


    async_base_address =
            address;

    async_length =
            length;

    async_low_byte =
            0U;

    async_retry_count =
            0U;


    async_state =
            BQ76942_ASYNC_WAIT_LOW_BYTE;


    if (!BQ76942_StartAsyncReadByte(
            address))
    {
        async_state =
                BQ76942_ASYNC_IDLE;


        return false;
    }


    return true;
}


/* ============================================================
 * SERVICE ASYNC DIRECT READ
 * ============================================================ */

static bool BQ76942_ServiceAsyncDirectRead(
        uint16_t *value)
{
    uint8_t expected_command;


    if (value == NULL)
    {
        read_state =
                BQ76942_READ_ERROR;


        return false;
    }


    if (spi_transfer_error ||
        BQ76942_AsyncTimedOut())
    {
        HAL_GPIO_WritePin(
                BQ2_CS_GPIO_Port,
                BQ2_CS_Pin,
                GPIO_PIN_SET);


        read_state =
                BQ76942_READ_ERROR;


        return false;
    }


    if (!spi_transfer_complete)
    {
        return false;
    }


    HAL_GPIO_WritePin(
            BQ2_CS_GPIO_Port,
            BQ2_CS_Pin,
            GPIO_PIN_SET);


    spi_transfer_complete =
            false;

    async_transaction_start_ms =
            0U;


    expected_command =
            BQ76942_SPI_READ_BIT |
            ((async_state ==
              BQ76942_ASYNC_WAIT_LOW_BYTE) ?
             async_base_address :
             (uint8_t)(async_base_address + 1U));


    if (!BQ76942_ResponseValid(
            expected_command,
            spi_rx))
    {
        if (async_retry_count >=
            BQ76942_SPI_MAX_RETRIES)
        {
            read_state =
                    BQ76942_READ_ERROR;


            return false;
        }


        async_retry_count++;


        if (!BQ76942_StartAsyncReadByte(
                (async_state ==
                 BQ76942_ASYNC_WAIT_LOW_BYTE) ?
                async_base_address :
                (uint8_t)(async_base_address + 1U)))
        {
            read_state =
                    BQ76942_READ_ERROR;
        }


        return false;
    }


    async_retry_count =
            0U;


    if (async_state ==
        BQ76942_ASYNC_WAIT_LOW_BYTE)
    {
        async_low_byte =
                spi_rx[1];


        if (async_length == 1U)
        {
            *value =
                    async_low_byte;


            return true;
        }


        async_state =
                BQ76942_ASYNC_WAIT_HIGH_BYTE;


        if (!BQ76942_StartAsyncReadByte(
                (uint8_t)(
                        async_base_address +
                        1U)))
        {
            read_state =
                    BQ76942_READ_ERROR;
        }


        return false;
    }


    *value =
            (uint16_t)async_low_byte |
            ((uint16_t)spi_rx[1] << 8);


    return true;
}


/* ============================================================
 * ASYNC TIMEOUT
 * ============================================================ */

static bool BQ76942_AsyncTimedOut(void)
{
    if (async_transaction_start_ms == 0U)
    {
        return false;
    }


    return
            ((HAL_GetTick() -
              async_transaction_start_ms) >
             BQ76942_SPI_RESPONSE_TIMEOUT_MS);
}
