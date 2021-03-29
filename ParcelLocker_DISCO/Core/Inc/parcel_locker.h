/** ************************************************************************
 *
 * @file      : parcel_locker.h
 * @copyright : Zasilkovna
 *
 * Created: Mar 28, 2021
 * Author:  Michal Veteska
 *
 *
 * @addtogroup parcel_locker
 * @brief Implementation parcel locker protocol.
 *
 * For module functionality, it must be initialized by the xv_prclcr_init
 * function and assigned a callback xv_prclcr_uart_rx_callback to receive
 * data via uart. The xv_prclcr_init function can contain a callback i
 * nput parameter of the TPRCLCR_CALLBACK type, which will be called if a
 * command to open / close a box occurs, or a NULL value. The user can set
 * the number of cabinets in the range 1-255 and the number of boxes 1-255
 * using define PRCLCR_CAINET_NUM and PRCLCR_NUM_OF_BOX_STATUS.
 *
 * ************************************************************************* */

#ifndef INC_PARCEL_LOCKER_H_
#define INC_PARCEL_LOCKER_H_

/* =============================================================
 * Inserted header files
 * ============================================================= */
#include <string.h>
/* Include main just for back compatibility with HAL lib..
 * User can change MCU setting and everything still by work.*/
#include "main.h"
/* -------------------------------------------------------------
 * 						 Public section
 * ------------------------------------------------------------- */

/* =============================================================
 * Definition of global variable that is adjustable also from other modules.
 * ============================================================= */

/* Define max number of box in one cabinet. */
#ifndef PRCLCR_NUM_OF_BOX_STATUS
#define PRCLCR_NUM_OF_BOX_STATUS    3U
#endif
/* Define max number of cabinet. */
#ifndef PRCLCR_CAINET_NUM
#define PRCLCR_CAINET_NUM			255U
#endif

/* Typedef of callback type for interface between parcel_locker module and code drive box magnet. */
typedef void (*TPRCLCR_CALLBACK)(uint8_t ppu8_box_status[PRCLCR_CAINET_NUM][PRCLCR_NUM_OF_BOX_STATUS]);

/* Typedef error state of function in module. */
typedef enum
{
	ERR_OK 	  = 0U,
	ERR_FAILD = 1U,
	ERR_BUSY  = 2U
}TPRCLCR_ERR;


/** Typedef structure for parcel locker command format instruction. */
typedef struct
{
	uint8_t	u8_cmd_head;
	uint8_t	u8_cab_addr;
	uint8_t	u8_box_num;
	uint8_t	u8_com_type;
	uint8_t	u8_par_qlt;
	uint8_t u8_par_m;
	uint8_t u8_cmd_cs;
	uint8_t u8_cmd_end;
}TPRCLCR_CMD_FORM_INST;



/* =============================================================
 * Definition of global functions or procedures that is adjustable also from other modules.
 * ============================================================= */
void xv_prclcr_uart_rx_callback(UART_HandleTypeDef *huart);
void xv_prclcr_init(TPRCLCR_CALLBACK pv_box_open_fnc);
void xv_prclcr_get_box_status(uint8_t u8_box_num[PRCLCR_CAINET_NUM][PRCLCR_NUM_OF_BOX_STATUS]);
/* -------------------------------------------------------------
 * 						 Private section
 * ------------------------------------------------------------- */
#ifdef __PARCEL_LOCKER_PRIVATE__

/* =============================================================
 * Definition of local variable, procedures, defines that is used only in this module.
 * ============================================================= */
/* Macro for clean register */
#define CLEAN_REG(x)				(x=0U)

/* Define TRUU FALSE */
#define TRUE						1U
#define FALSE						0U

/* Define length of packet without head and end. */
#define PRCLCR_CMD_FORM_INST_LEN	6U

#define PRCLCR_START_CMD			0x21U
#define PRCLCR_END_CMD				0x16U

#define PRCLCR_CS_MASK				0xFFU

/* Typedef of all  which could occur. */
typedef enum
{
	BOX_DOOR_CONTROL   = 0x01U,
	BOX_STATUS_QUERY   = 0x02U,
	BOX_DOOR_STATUS    = 0x07U,
	FIRMWARE_VERSION   = 0x08U,
	COMUNICATION_CHECK = 0x09U
}TPRCLCR_CMD_TYPE;

/* =============================================================
 * Definition of local functions or procedures that is used only in this module.
 * ============================================================= */
TPRCLCR_ERR err_prclcr_packet_decode(uint8_t u8_rx_data);
TPRCLCR_ERR err_prclcr_packet_save(uint8_t u8_data, uint8_t u8_rst_st);
void v_prclcr_cmd_set(UART_HandleTypeDef *huart);
TPRCLCR_ERR err_prclcr_box_door_ctr(void);
void v_prclcr_box_door_satus(void);

#endif


#endif /* INC_PARCEL_LOCKER_H_ */
