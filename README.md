# MUSSDE_SDTR
Ejercicios de clase SDTR


Función CAN

```
void send_CAN_uint32_t(uint32_t value){
	CAN_TxHeaderTypeDef pHeader;
	CAN_RxHeaderTypeDef pRxHeader;
	pHeader.DLC=5;
	pHeader.IDE=CAN_ID_STD;
	pHeader.RTR=CAN_RTR_DATA;
	pHeader.StdId=0x2FF;
	uint32_t TxMailbox;
	
	uint8_t buffer[5] = {};
	buffer[0] = 255;
        buffer[1] = (uint8_t) ((value & 0xFF000000) >> 24);
	buffer[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
	buffer[3] = (uint8_t) ((value & 0x0000FF00) >> 8);
	buffer[4] = (uint8_t) ((value & 0x000000FF));
	HAL_CAN_AddTxMessage(&hcan1, &pHeader, buffer, &TxMailbox);
}
```


