#include "application/communication_select.h"

#include <stdio.h>

#include "main.h"
#include "mb.h"
#include "lwip.h"


static ErrorCode_t activate(SystemCommunicationMode_t mode);
static ErrorCode_t deactivate(SystemCommunicationMode_t mode);

static ErrorCode_t enable_modbus_rtu(void);
static ErrorCode_t disable_modbus_rtu(void);

static ErrorCode_t enable_modbus_tcp(void);
static ErrorCode_t disable_modbus_tcp(void);

static ErrorCode_t enable_profibus(void);
static ErrorCode_t disable_profibus(void);

static ErrorCode_t convert_modbus_errorcode(eMBErrorCode result);


ErrorCode_t communication_select_initialize(void)
{
	SystemCommunicationMode_t mode;
	ErrorCode_t result;

	mode = system_config_get_communication_mode();
	result = activate(mode);
	if (result != ERROR_CODE_NONE)
	{
		printf(
				"%d: Industrial communication initialize error\r\n",
				(int)result
		);
		return result;
	}

	switch (mode)
	{
		case SYSTEM_COMMUNICATION_MODE_MODBUS_RTU:
			printf("Modbus RTU initialized\r\n");
			break;

		case SYSTEM_COMMUNICATION_MODE_MODBUS_TCP:
			printf("Modbus TCP initialized\r\n");
			break;

		case SYSTEM_COMMUNICATION_MODE_PROFIBUS:
			printf("PROFIBUS initialized\r\n");
			break;

		default:
			break;
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t communication_select_mode(SystemCommunicationMode_t mode)
{
	SystemCommunicationMode_t current_mode;
	ErrorCode_t result;
	ErrorCode_t restore_result;

	current_mode = system_config_get_communication_mode();
	if (mode == current_mode)
	{
		return ERROR_CODE_NONE;
	}

	result = deactivate(current_mode);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = activate(mode);
	if (result != ERROR_CODE_NONE)
	{
		restore_result = activate(current_mode);
		if (restore_result != ERROR_CODE_NONE)
		{
			printf(
					"%d: Industrial communication restore error\r\n",
					(int)restore_result
			);
		}
		return result;
	}

	system_config_set_communication_mode(mode);
	return result;
}

ErrorCode_t communication_select_process(void)
{
	ErrorCode_t result;

	switch (system_config_get_communication_mode())
	{
		case SYSTEM_COMMUNICATION_MODE_MODBUS_RTU:
			result = convert_modbus_errorcode(eMBPoll());
			break;

		case SYSTEM_COMMUNICATION_MODE_MODBUS_TCP:
			// MX_LWIP_Process();
			/* Modbus TCP is not implemented yet. */
			result = ERROR_CODE_NONE;
			break;

		case SYSTEM_COMMUNICATION_MODE_PROFIBUS:
			/* PROFIBUS is not implemented yet. */
			result = ERROR_CODE_NONE;
			break;

		default:
			result = ERROR_CODE_COMMUNICATION_SELECT_ILLEGAL_MODE;
			break;
	}

	if (result != ERROR_CODE_NONE)
	{
		printf(
			"%d: Industrial communication process error\r\n",
			(int)result
		);
		return result;
	}

	return result;
}

static ErrorCode_t activate(SystemCommunicationMode_t mode)
{
	switch (mode)
	{
		case SYSTEM_COMMUNICATION_MODE_MODBUS_RTU:
			return enable_modbus_rtu();

		case SYSTEM_COMMUNICATION_MODE_MODBUS_TCP:
			return enable_modbus_tcp();

		case SYSTEM_COMMUNICATION_MODE_PROFIBUS:
			return enable_profibus();

		default:
			return ERROR_CODE_COMMUNICATION_SELECT_ILLEGAL_MODE;
	}
}

static ErrorCode_t deactivate(SystemCommunicationMode_t mode)
{
	switch (mode)
	{
		case SYSTEM_COMMUNICATION_MODE_MODBUS_RTU:
			return disable_modbus_rtu();

		case SYSTEM_COMMUNICATION_MODE_MODBUS_TCP:
			return disable_modbus_tcp();

		case SYSTEM_COMMUNICATION_MODE_PROFIBUS:
			return disable_profibus();

		default:
			return ERROR_CODE_COMMUNICATION_SELECT_ILLEGAL_MODE;
	}
}

static ErrorCode_t enable_modbus_rtu(void)
{
	ModbusRTUParity_t parity;
	eMBParity modbus_parity;
	eMBErrorCode modbus_result;

	parity = system_config_get_modbus_rtu_parity();
	switch (parity)
	{
		case MODBUS_RTU_PARITY_NONE:
			modbus_parity = MB_PAR_NONE;
			break;

		case MODBUS_RTU_PARITY_ODD:
			modbus_parity = MB_PAR_ODD;
			break;

		case MODBUS_RTU_PARITY_EVEN:
			modbus_parity = MB_PAR_EVEN;
			break;

		default:
			return ERROR_CODE_MODBUS_RTU_ILLEGAL_PARAM;
	}

	modbus_result = eMBInit(
			MB_RTU,
			system_config_get_modbus_rtu_slave_address(),
			system_config_get_modbus_rtu_port(),
			system_config_get_modbus_rtu_baud_rate(),
			modbus_parity
	);
	if (modbus_result != MB_ENOERR)
	{
		return convert_modbus_errorcode(modbus_result);
	}

	modbus_result = eMBEnable();
	return convert_modbus_errorcode(modbus_result);
}

static ErrorCode_t disable_modbus_rtu(void)
{
	eMBErrorCode modbus_result;

	modbus_result = eMBDisable();
	if (modbus_result != MB_ENOERR)
	{
		return convert_modbus_errorcode(modbus_result);
	}

	modbus_result = eMBClose();
	return convert_modbus_errorcode(modbus_result);
}

static ErrorCode_t enable_modbus_tcp(void)
{
	HAL_GPIO_WritePin(
			ETH_PHY_RESET_GPIO_Port,
			ETH_PHY_RESET_Pin,
			GPIO_PIN_RESET
	);
	HAL_Delay(50);
	HAL_GPIO_WritePin(
			ETH_PHY_RESET_GPIO_Port,
			ETH_PHY_RESET_Pin,
			GPIO_PIN_SET
	);
	HAL_Delay(50);
	// MX_LWIP_Init();
	/* Modbus TCP enable is not implemented yet. */
	return ERROR_CODE_NONE;
}

static ErrorCode_t disable_modbus_tcp(void)
{
	/* Modbus TCP disable is not implemented yet. */
	return ERROR_CODE_NONE;
}

static ErrorCode_t enable_profibus(void)
{
	/* PROFIBUS enable is not implemented yet. */
	return ERROR_CODE_NONE;
}

static ErrorCode_t disable_profibus(void)
{
	/* PROFIBUS disable is not implemented yet. */
	return ERROR_CODE_NONE;
}

static ErrorCode_t convert_modbus_errorcode(eMBErrorCode result)
{
	switch (result)
	{
		case MB_ENOERR:
			return ERROR_CODE_NONE;

		case MB_ENOREG:
			return ERROR_CODE_MODBUS_RTU_REGISTER;

		case MB_EINVAL:
			return ERROR_CODE_MODBUS_RTU_ILLEGAL_PARAM;

		case MB_EPORTERR:
			return ERROR_CODE_MODBUS_RTU_PORT;

		case MB_ENORES:
			return ERROR_CODE_MODBUS_RTU_RESOURCE;

		case MB_EIO:
			return ERROR_CODE_MODBUS_RTU_IO;

		case MB_EILLSTATE:
			return ERROR_CODE_MODBUS_RTU_STATE;

		case MB_ETIMEDOUT:
			return ERROR_CODE_MODBUS_RTU_TIMEOUT;

		default:
			return ERROR_CODE_UNKNOWN;
	}
}
