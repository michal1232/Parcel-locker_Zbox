/** ************************************************************************
 * @file      : parcel_locker.c
 * @copyright : Zasilkovna
 *
 * Created: Mar 28, 2021
 * Author:  Michal Veteska
 *
 *
 * @addtogroup parcel_locker
 * @brief Implementation parcel locker protocol.
 *
 *
 * ************************************************************************* */


/* =============================================================
 * Inserted header files
 * ============================================================= */
#define __PARCEL_LOCKER_PRIVATE__
#include "parcel_locker.h"

/* =============================================================
 * Definition of global, defines variable that is adjustable also from other modules.
 * ============================================================= */

/* =============================================================
 * Definition of local variable that is used only in this module.
 * ============================================================= */

/** Local static variable for drive state machine. */
static TPRCLCR_CMD_FORM_INST ls_cmd_form_inst_rx;
/** Local static variable for calculate data from incoming packet. */
static uint32_t 			 lu32_cs_rx = 1U;
/** Local static variable of error packet. */
static uint8_t 		         lu8_err_reply[8U] = {0x21U, 0x00U, 0x00U, 0x0AU, 0x01U, 0x00U, 0x2CU, 0x16U};
/** Local static variable of box array*/
static uint8_t				 lu8_box_status[PRCLCR_CAINET_NUM][PRCLCR_NUM_OF_BOX_STATUS];
/** Local static variable of tx extended packet. */
static uint8_t 				 lu8_cmd_form_inst_expand[7U + PRCLCR_NUM_OF_BOX_STATUS];

static TPRCLCR_CALLBACK		 lp_box_run_function = NULL;


/* =============================================================
 * Definition of global functions or procedures that is adjustable also from other modules.
 * ============================================================= */
/**
 * @brief Callback for usart2 rx interrupt.
 *
 * This function mediates callback interrupts from the USART2 peripheral. If a 1-byte reception occurs, 
 * the processor is awake and the function is called. The function is set as a callback in the main 
 * initialization and stored in the huart2 structure. The callback function first waits and decode 
 * the packet, which has to be received. If the received packet is decoded, the processing v_prclcr_cmd_set
 * function is started. If the received packet is damaged, an error message is sent back.
 *
 * @param  [in] huart		pointer to uart structure, which one provide interrupt
 */
void xv_prclcr_uart_rx_callback(UART_HandleTypeDef *huart)
{
	/* Read data from RDR register.*/
	uint8_t u8_rx_data = READ_REG(huart->Instance->RDR);

	/* Call packet decode function. If result is ERR_BUSY nothing do. */
	TPRCLCR_ERR enu_err = err_prclcr_packet_decode(u8_rx_data);

	/* If packet is valid run state machine of decoding and answer. */
	if(enu_err == ERR_OK) {
		v_prclcr_cmd_set(huart);
	}
	/* If packet is not valid error answer is send. */
	else if(enu_err == ERR_FAILD) {
		HAL_UART_Transmit_DMA(huart, lu8_err_reply, sizeof(lu8_err_reply));
	}
}

/**
 * @brief Initialize function of parcel locker.
 *
 * This function clean box status variable lu8_box_status.
 * It is call in main function. It also initializes the interface (callback),
 * which is called if there was a request to open / close the box. Opening / closing the box 
 * is not implemented in this module.
 */
void xv_prclcr_init(TPRCLCR_CALLBACK pv_box_open_fnc)
{
	uint16_t i, j;

	/* Clean box status array on start. */
	for(i = 0; i <= PRCLCR_CAINET_NUM; i++)
	{
		for(j = 0; j < PRCLCR_NUM_OF_BOX_STATUS; j++)
		{
			lu8_box_status[i][j] = 0;
		}
	}

	/* Set callback function which is run when open/close command income. */
	lp_box_run_function = pv_box_open_fnc;
}

/**
 *  @brief Function set copy of lu8_box_status to u8_box_num
 *
 *  This function is use as interface to another module.
 *
 *  @param [out] u8_box_num		Output data which copy from lu8_box_status
 */
void xv_prclcr_get_box_status(uint8_t u8_box_num[PRCLCR_CAINET_NUM][PRCLCR_NUM_OF_BOX_STATUS])
{
	uint8_t i,j;
	for(i = 0U; i < PRCLCR_CAINET_NUM; i++)
	{
		for(j = 0U; j < PRCLCR_NUM_OF_BOX_STATUS; j++)
		{
			u8_box_num[i][j] = lu8_box_status[i][j];
		}
	}
}

//TODO: Implement function which return status of box.
/* =============================================================
 * Definition of local functions or procedures that is used only in this module.
 * ============================================================= */

/**
* @brief Packet decode function.
*
* This function includes a state machine for receiving packet decoding. 
* Incoming decoded messages are stored in the ls_cmd_form_inst_rx structure. 
* The parameters u8_cmd_head and u8_cmd_end serve as locks for packet decoding 
* and do not contain data but a value indicating that the sending of the message 
* has been started / stopped. The state machine has three states: waiting to start, 
* writing a packet, ending a packet.
*
* @param  [in] u8_rx_data		Recieved data.
* @retval ERR_OK				Packet is complet and valid
*         ERR_FAILD				Packet is not valid 
*         ERR_BUSY				Packet incoming.		
*/
TPRCLCR_ERR err_prclcr_packet_decode(uint8_t u8_rx_data)
{
	/*Decode incoming packet. */
	if((u8_rx_data == PRCLCR_START_CMD) && (ls_cmd_form_inst_rx.u8_cmd_head == FALSE))
	{
		/* Set cmd header to true for lock machine and set it to wait for data */
		ls_cmd_form_inst_rx.u8_cmd_head = TRUE;

		/* Clean checksum */
		CLEAN_REG(lu32_cs_rx);
		/* Add new data to cs */
		lu32_cs_rx += u8_rx_data;
	}
	else if((u8_rx_data == PRCLCR_END_CMD) && (ls_cmd_form_inst_rx.u8_cmd_head == FALSE))
	{
		/* Machine in end state. Reset counter save state machine */
		(void)err_prclcr_packet_save(0U, TRUE);

		/* Check mask */
		if((uint8_t)(lu32_cs_rx & PRCLCR_CS_MASK) == ls_cmd_form_inst_rx.u8_cmd_cs)
		{
			/* Reset decote state machine. */
			ls_cmd_form_inst_rx.u8_cmd_head = FALSE;
			ls_cmd_form_inst_rx.u8_cmd_end = TRUE;
			CLEAN_REG(lu32_cs_rx);

			/* Run answer process */
			return ERR_OK;
		}
		else
		{
			/* Error frame received */
			return ERR_FAILD;
		}
	}
	else if(ls_cmd_form_inst_rx.u8_cmd_head == TRUE)
	{
		/* Machine in save data state. Check if data can by write*/
		if(err_prclcr_packet_save(u8_rx_data, FALSE) != ERR_OK)
		{
			/* If not reset machine */
			ls_cmd_form_inst_rx.u8_cmd_head = FALSE;
			ls_cmd_form_inst_rx.u8_cmd_end = FALSE;
			return ERR_FAILD;
		}
	}
	else {/* Empty else*/}

	return ERR_BUSY;
}

/**
* @brief Packet save state machine function.
*
* The function stores data in the ls_cmd_form_inst_rx structure. If an overflow of the maximum 
* number of data in the packet structure occurs, it is declared invalid.
*
* @param  [in] u8_data          Data to store.
* @param  [in] u8_rst_st        TRUE/FALSE - RESET/NOT RESET state machine.
* @retval ERR_OK				Packet is complet and valid, state machine successful reset.
*         ERR_FAILD				Packet is not valid.
*/
TPRCLCR_ERR err_prclcr_packet_save(uint8_t u8_data, uint8_t u8_rst_st)
{
	static uint8_t u8_data_cnt = 0;
	
	/* Reset state machine if u8_rst_st is set. */
	if(u8_rst_st == TRUE)
	{
		/* Call macro to clean state counter. */
		CLEAN_REG(u8_data_cnt);
		return ERR_OK;
	}
	/* Check if packet is still valid. */
	else if(u8_data_cnt == PRCLCR_CMD_FORM_INST_LEN)
	{
		/* Call macro to clean state counter. */
		CLEAN_REG(u8_data_cnt);
		return ERR_FAILD;
	}
	/* Save state machine function. */
	else
	{
		switch(u8_data_cnt)
		{
		case 0U: ls_cmd_form_inst_rx.u8_cab_addr = u8_data;
				 lu32_cs_rx += u8_data;
				 break;
		case 1U: ls_cmd_form_inst_rx.u8_box_num = u8_data;
				 lu32_cs_rx += u8_data;
				 break;
		case 2U: ls_cmd_form_inst_rx.u8_com_type = u8_data;
		         lu32_cs_rx += u8_data;
				 break;
		case 3U: ls_cmd_form_inst_rx.u8_par_qlt = u8_data;
		         lu32_cs_rx += u8_data;
				 break;
		case 4U: ls_cmd_form_inst_rx.u8_par_m = u8_data;
				 lu32_cs_rx += u8_data;
				 break;
		case 5U: ls_cmd_form_inst_rx.u8_cmd_cs = u8_data;
				 /* End of save. Drive err_prclcr_packet_decode function */
				 ls_cmd_form_inst_rx.u8_cmd_head = FALSE;
				 break;
		default: break; /* Empty default*/
		}

		u8_data_cnt++;

		return ERR_OK;
	}
	return ERR_OK;
}

/**
 * @brief Command set function.
 *
 * The function performs the given action based on the incoming command.
 *
 * @param  [in]  *huart			Pointer to current used uart.
 *
 */

void v_prclcr_cmd_set(UART_HandleTypeDef *huart)
{
	TPRCLCR_CMD_FORM_INST s_rx_response = ls_cmd_form_inst_rx;

	switch(ls_cmd_form_inst_rx.u8_com_type)
	{
	case BOX_DOOR_CONTROL:
		/* Set local data lu8_box_status. If data can by set m par is set to 1. */
		if(err_prclcr_box_door_ctr() == ERR_OK) {
			s_rx_response.u8_par_m = TRUE;
		}
		else {
			s_rx_response.u8_par_m = FALSE;
		}

		s_rx_response.u8_cmd_head = PRCLCR_START_CMD;
		s_rx_response.u8_cmd_end  = PRCLCR_END_CMD;

		/* Send response packet. */
//TODO: It is written in the assignment that the slave has to answer within 1s.
//TODO: I'm not sure if it's possible to open the box so quickly.
//TODO: Therefore, a response is sent first, and then does the box open.
		HAL_UART_Transmit_DMA(huart, (uint8_t *)&s_rx_response, 8U);

		/* If function open/close is implemented, it will by run.*/
		if(lp_box_run_function != NULL)
		{
			lp_box_run_function(lu8_box_status);
		}

		break;
	case BOX_STATUS_QUERY:    /* No implemented */
		break;
	case BOX_DOOR_STATUS:
		/* Create respond packet and send it. */
		v_prclcr_box_door_satus();
		HAL_UART_Transmit_DMA(huart, lu8_cmd_form_inst_expand, sizeof(lu8_cmd_form_inst_expand));
		break;
	case FIRMWARE_VERSION:    /* No implemented */
		break;
	case COMUNICATION_CHECK:  /* No implemented */
		break;
	}
}

/**
 * @brief Function fill box value array.
 *
 * Based on the incoming cabinet address and box number, the function stores the data
 * on the opening / closing of the given box. If the cabinet address is set to 0,
 * the box number is set to all cabinets. If box number is set to 0, nothing is done
 * and the answer is about impossible opening / closing of boxes.
 *
 * @retval ERR_OK				Box number is set to open/close status
 *         ERR_FAILD			Box number cannot by set.
 *
 */
TPRCLCR_ERR err_prclcr_box_door_ctr(void)
{
	/* Calculate which bit in lu8_box_status has to by set/reset */
	uint8_t u8_msg_box_sel = (uint8_t)((ls_cmd_form_inst_rx.u8_box_num - 1U) / 8U);
	uint8_t u8_msg_box_mod = (uint8_t)((ls_cmd_form_inst_rx.u8_box_num - 1U) % 8U);

	/* Check if number of box is not bigger as in code define. */
	if(u8_msg_box_sel >= PRCLCR_NUM_OF_BOX_STATUS)
	{
		return ERR_FAILD;
	}

	/* Broadcast for all cabinet. */
	if(ls_cmd_form_inst_rx.u8_cab_addr == 0U)
	{
		if(ls_cmd_form_inst_rx.u8_box_num == 0U)
		{
			/* Never can by set all box in cabinet at once. */
			return ERR_FAILD;
		}
		else
		{
			uint8_t i;
			for(i = 0; i < PRCLCR_CAINET_NUM; i++)
			{
				lu8_box_status[i][u8_msg_box_sel] &= ~(uint8_t)(1U << u8_msg_box_mod);
				lu8_box_status[i][u8_msg_box_sel] |= (uint8_t)(ls_cmd_form_inst_rx.u8_par_m << u8_msg_box_mod);
			}
		}
	}
	/* Set/reset current one box. */
	else
	{
		if(ls_cmd_form_inst_rx.u8_box_num == 0U)
		{
			/* Never can by set all box in cabinet at once. */
			return ERR_FAILD;
		}
		else
		{
			lu8_box_status[ls_cmd_form_inst_rx.u8_cab_addr - 1U][u8_msg_box_sel] &= ~(uint8_t)(1U << u8_msg_box_mod);
			lu8_box_status[ls_cmd_form_inst_rx.u8_cab_addr - 1U][u8_msg_box_sel] |= (uint8_t)(ls_cmd_form_inst_rx.u8_par_m << u8_msg_box_mod);
		}
	}
	return ERR_OK;
}

/**
 * @brief Function of setting door status to sending array.
 *
 * The function sets the field for sending the status of the required cabinet.
 * The number of boxes sent is given by the constant PRCLCR_NUM_OF_BOX_STATUS.
 */
void v_prclcr_box_door_satus(void)
{
	uint8_t i;
	uint32_t u32_packet_cs = 0;

	lu8_cmd_form_inst_expand[0U] = PRCLCR_START_CMD;
	lu8_cmd_form_inst_expand[1U] = ls_cmd_form_inst_rx.u8_cab_addr;
	lu8_cmd_form_inst_expand[2U] = ls_cmd_form_inst_rx.u8_box_num;
	lu8_cmd_form_inst_expand[3U] = ls_cmd_form_inst_rx.u8_com_type;
	lu8_cmd_form_inst_expand[4U] = PRCLCR_NUM_OF_BOX_STATUS;

	/* Set data of box_status to array. */
	for(i = 0; i < PRCLCR_NUM_OF_BOX_STATUS; i++)
	{
		lu8_cmd_form_inst_expand[5U + i] = lu8_box_status[ls_cmd_form_inst_rx.u8_cab_addr - 1U][i];
	}

	/* Calculate new cs */
//TODO: It is written in the assignment that the slave should count cs (page 2, point 2 -> 1 CS :)
//TODO: but in the example of the answer (page 6) cs is not calculated but only forwarded.
	for(i = 0; i < (5U + PRCLCR_NUM_OF_BOX_STATUS); i++)
	{
		u32_packet_cs += lu8_cmd_form_inst_expand[i];
	}

	lu8_cmd_form_inst_expand[5U + PRCLCR_NUM_OF_BOX_STATUS] = (uint8_t)(u32_packet_cs & 0xFF);
	lu8_cmd_form_inst_expand[6U + PRCLCR_NUM_OF_BOX_STATUS] = PRCLCR_END_CMD;
}



