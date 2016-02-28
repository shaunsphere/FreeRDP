/**
 * FreeRDP: A Remote Desktop Protocol Implementation
 * RemoteFX USB Redirection
 *
 * Copyright 2012 Atrust corp.
 * Copyright 2012 Alfred Liu <alfred.liu@atruscorp.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <winpr/sysinfo.h>

#include "urbdrc_types.h"
#include "data_transfer.h"

static BOOL usb_process_get_port_status(IUDEVICE* pdev, wStream* data)
{
	int bcdUSB = pdev->query_device_descriptor(pdev, BCD_USB);

	if (!Stream_EnsureRemainingCapacity(data, 4))
		return FALSE;

	switch (bcdUSB)
	{
	case USB_v1_0:
		Stream_Write_UINT32(data, 0x303);
		break;

	case USB_v1_1:
		Stream_Write_UINT32(data, 0x103);
		break;

	case USB_v2_0:
		Stream_Write_UINT32(data, 0x503);
		break;

	default:
		Stream_Write_UINT32(data, 0x503);
		break;
	}

	return TRUE;
}

#if ISOCH_FIFO

static int func_check_isochronous_fds(IUDEVICE* pdev)
{
	int ret = 0;
	BYTE* data_temp;
	UINT32 size_temp, process_times = 2;
	ISOCH_CALLBACK_QUEUE* isoch_queue = NULL;
	ISOCH_CALLBACK_DATA* isoch = NULL;
	URBDRC_CHANNEL_CALLBACK* callback;

	isoch_queue = (ISOCH_CALLBACK_QUEUE*) pdev->get_isoch_queue(pdev);

	while (process_times)
	{
		process_times--;

		if (isoch_queue == NULL || !pdev)
			return -1;

		if (WaitForSingleObject(isoch_queue->isoch_loading, INFINITE) != WAIT_OBJECT_0)
			return -1;

		if (isoch_queue->head == NULL)
		{
			if (!ReleaseMutex(isoch_queue->isoch_loading))
				return -1;
			continue;
		}
		else
			isoch = isoch_queue->head;

		if (!isoch || !isoch->out_data)
		{
			if (!ReleaseMutex(isoch_queue->isoch_loading))
				return -1;
			continue;
		}
		else
		{
			callback = (URBDRC_CHANNEL_CALLBACK*) isoch->callback;
			size_temp = isoch->out_size;
			data_temp = isoch->out_data;

			ret = isoch_queue->unregister_data(isoch_queue, isoch);

			if (!ret)
				WLog_DBG(TAG, "isoch_queue_unregister_data: Not found isoch data!!");

			if (WaitForSingleObject(isoch_queue->isoch_loading, INFINITE) != WAIT_OBJECT_0)
				return -1;

			if (pdev && !pdev->isSigToEnd(pdev))
			{
				callback->channel->Write(callback->channel, size_temp, data_temp, NULL);
				free(data_temp);
			}
		}
	}

	return 0;
}

#endif

static int urbdrc_process_register_request_callback(URBDRC_CHANNEL_CALLBACK* callback,
													wStream* data, IUDEVMAN* udevman, UINT32 UsbDevice)
{
	IUDEVICE* pdev;
	UINT32 NumRequestCompletion = 0;
	UINT32 RequestCompletion = 0;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	if (Stream_GetRemainingLength(data) >= 8)
	{
		Stream_Read_UINT32(data, NumRequestCompletion); /** must be 1 */
		/** RequestCompletion:
		*   unique Request Completion interface for the client to use */
		Stream_Read_UINT32(data, RequestCompletion);
		pdev->set_ReqCompletion(pdev, RequestCompletion);
	}
	else /** Unregister the device */
	{
		Stream_Read_UINT32(data, RequestCompletion);

		if (1)//(pdev->get_ReqCompletion(pdev) == RequestCompletion)
		{
			/** The wrong driver may also receive this message, So we
			 *  need some time(default 3s) to check the driver or delete
			 *  it */
			Sleep(3);
			callback->channel->Write(callback->channel, 0, NULL, NULL);
			pdev->SigToEnd(pdev);
		}
	}

	return 0;
}

static int urbdrc_process_cancel_request(wStream* data, IUDEVMAN* udevman, UINT32 UsbDevice)
{
	IUDEVICE* pdev;
	UINT32 CancelId;
	int error = 0;

	if (Stream_GetRemainingLength(data) < 4)
		return ERROR_INVALID_DATA;

	Stream_Read_UINT32(data, CancelId); /** RequestId */

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	error = pdev->cancel_transfer_request(pdev, CancelId);

	return error;
}

static int urbdrc_process_retract_device_request(wStream* data, IUDEVMAN* udevman, UINT32 UsbDevice)
{
	int rc = -1;
	UINT32 Reason;

	if (Stream_GetRemainingLength(data) < 4)
		return ERROR_INVALID_DATA;

	Stream_Read_UINT32(data, Reason); /** Reason */

	switch (Reason)
	{
	case UsbRetractReason_BlockedByPolicy:
		WLog_DBG(TAG, "UsbRetractReason_BlockedByPolicy: now it is not support");
		break;

	default:
		WLog_DBG(TAG, "urbdrc_process_retract_device_request: Unknown Reason %d", Reason);
		break;
	}

	return rc;
}

static int urbdrc_process_io_control(URBDRC_CHANNEL_CALLBACK* callback, wStream* data,
									 UINT32 MessageId, IUDEVMAN * udevman, UINT32 UsbDevice)
{
	size_t pos;
	IUDEVICE* pdev;
	UINT32 InterfaceId;
	UINT32 IoControlCode;
	UINT32 InputBufferSize;
	UINT32 OutputBufferSize;
	UINT32 RequestId;
	UINT32 usbd_status = USBD_STATUS_SUCCESS;
	wStream* OutputBuffer;
	int success = 0;

	if (Stream_GetRemainingLength(data) < 8)
		return ERROR_INVALID_DATA;

	Stream_Read_UINT32(data, IoControlCode);
	Stream_Read_UINT32(data, InputBufferSize);

	if (Stream_GetRemainingLength(data) < 8 + InputBufferSize)
		return ERROR_INVALID_DATA;

	Stream_Seek(data, InputBufferSize);
	Stream_Read_UINT32(data, OutputBufferSize);
	Stream_Read_UINT32(data, RequestId);

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	/**  process */
	OutputBuffer = Stream_New(NULL, OutputBufferSize + 28);
	if (!OutputBuffer)
		return ERROR_OUTOFMEMORY;

	Stream_Seek(OutputBuffer, 28);
	switch (IoControlCode)
	{
	case IOCTL_INTERNAL_USB_SUBMIT_URB:  /** 0x00220003 */
		WLog_DBG(TAG, "ioctl: IOCTL_INTERNAL_USB_SUBMIT_URB");
		WLog_ERR(TAG,  " Function IOCTL_INTERNAL_USB_SUBMIT_URB: Unchecked");
		break;

	case IOCTL_INTERNAL_USB_RESET_PORT:  /** 0x00220007 */
		WLog_WARN(TAG, "ioctl: IOCTL_INTERNAL_USB_RESET_PORT");
		WLog_ERR(TAG,  " Function IOCTL_INTERNAL_USB_RESET_PORT: Unchecked");
		break;

	case IOCTL_INTERNAL_USB_GET_PORT_STATUS: /** 0x00220013 */
		success = pdev->query_device_port_status(pdev, &usbd_status, OutputBuffer);

		if (success)
		{
			if (pdev->isExist(pdev) == 0)
				Stream_Write_UINT32(OutputBuffer, 0);
			else if (!usb_process_get_port_status(pdev, OutputBuffer))
			{
				Stream_Free(OutputBuffer, TRUE);
				return -1;
			}
		}

		break;

	case IOCTL_INTERNAL_USB_CYCLE_PORT:  /** 0x0022001F */
		WLog_DBG(TAG, "ioctl: IOCTL_INTERNAL_USB_CYCLE_PORT");
		WLog_ERR(TAG,  " Function IOCTL_INTERNAL_USB_CYCLE_PORT: Unchecked");
		break;

	case IOCTL_INTERNAL_USB_SUBMIT_IDLE_NOTIFICATION: /** 0x00220027 */
		WLog_DBG(TAG, "ioctl: IOCTL_INTERNAL_USB_SUBMIT_IDLE_NOTIFICATION");
		WLog_ERR(TAG,  " Function IOCTL_INTERNAL_USB_SUBMIT_IDLE_NOTIFICATION: Unchecked");
		break;

	default:
		WLog_DBG(TAG, "urbdrc_process_io_control: unknown IoControlCode 0x%X", IoControlCode);
		free(OutputBuffer);
		return ERROR_INVALID_OPERATION;
		break;
	}

	pos = Stream_GetPosition(OutputBuffer);
	Stream_SetPosition(OutputBuffer, 0);

	Stream_Write_UINT32(OutputBuffer, InterfaceId); /** interface */
	Stream_Write_UINT32(OutputBuffer, MessageId); /** message id */
	Stream_Write_UINT32(OutputBuffer, IOCONTROL_COMPLETION); /** function id */
	Stream_Write_UINT32(OutputBuffer, RequestId); /** RequestId */
	Stream_Write_UINT32(OutputBuffer, USBD_STATUS_SUCCESS); /** HResult */
	Stream_Write_UINT32(OutputBuffer, OutputBufferSize); /** Information */
	Stream_Write_UINT32(OutputBuffer, OutputBufferSize); /** OutputBufferSize */

	Stream_SetPosition(OutputBuffer, pos);
	Stream_SealLength(OutputBuffer);

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(OutputBuffer),
								 Stream_Buffer(OutputBuffer), NULL);

	Stream_Free(OutputBuffer, TRUE);

	return CHANNEL_RC_OK;
}

static int urbdrc_process_internal_io_control(URBDRC_CHANNEL_CALLBACK* callback,
											  wStream* data, UINT32 MessageId,
											  IUDEVMAN* udevman, UINT32 UsbDevice)
{
	IUDEVICE* pdev;
	wStream* out_data;
	UINT32 out_size, IoControlCode, InterfaceId, InputBufferSize;
	UINT32 OutputBufferSize, RequestId, frames;

	if (Stream_GetRemainingLength(data) < 16)
		return -1;

	Stream_Read_UINT32(data, IoControlCode);
	Stream_Read_UINT32(data, InputBufferSize);
	Stream_Read_UINT32(data, OutputBufferSize);
	Stream_Read_UINT32(data, RequestId);

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	frames = GetTickCount();

	out_size = 32;
	out_data = Stream_New(NULL, out_size);
	if (!out_data)
		return -1;

	Stream_Write_UINT32(out_data, InterfaceId); /** interface */
	Stream_Write_UINT32(out_data, MessageId); /** message id */
	Stream_Write_UINT32(out_data, IOCONTROL_COMPLETION); /** function id */
	Stream_Write_UINT32(out_data, RequestId); /** RequestId */
	Stream_Write_UINT32(out_data, 0); /** HResult */
	Stream_Write_UINT32(out_data, 4); /** Information */
	Stream_Write_UINT32(out_data, 4); /** OutputBufferSize */
	Stream_Write_UINT32(out_data, frames); /** OutputBuffer */
	Stream_SealLength(out_data);

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(out_data),
								 Stream_Buffer(out_data), NULL);

	Stream_Free(out_data, TRUE);

	return 0;
}

static int urbdrc_process_query_device_text(URBDRC_CHANNEL_CALLBACK* callback,
											wStream* data, UINT32 MessageId,
											IUDEVMAN* udevman, UINT32 UsbDevice)
{
	IUDEVICE* pdev;
	UINT32 out_size;
	UINT32 InterfaceId;
	UINT32 TextType;
	UINT32 LocaleId;
	UINT32 bufferSize = 1024;
	wStream* out_data;
	BYTE DeviceDescription[bufferSize];

	if (Stream_GetRemainingLength(data) < 8)
		return -1;
	Stream_Read_UINT32(data, TextType);
	Stream_Read_UINT32(data, LocaleId);

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	pdev->control_query_device_text(pdev, TextType, LocaleId, &bufferSize, DeviceDescription);

	InterfaceId = ((STREAM_ID_STUB << 30) | UsbDevice);

	out_size = 16 + bufferSize;

	if (bufferSize != 0)
		out_size += 2;

	out_data = Stream_New(NULL, out_size);
	if (!out_data)
		return -1;

	Stream_Write_UINT32(out_data, InterfaceId); /** interface */
	Stream_Write_UINT32(out_data, MessageId); /** message id */

	if (bufferSize != 0)
	{
		Stream_Write_UINT32(out_data, (bufferSize/2)+1); /** cchDeviceDescription */
		Stream_Write(out_data, DeviceDescription, bufferSize);
		Stream_Write_UINT16(out_data, 0x0000);
	}
	else
		Stream_Write_UINT32(out_data, 0); /** cchDeviceDescription */

	Stream_Write_UINT32(out_data, 0); /** HResult */
	Stream_SealLength(out_data);

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(out_data),
								 Stream_Buffer(out_data), NULL);

	Stream_Free(out_data, TRUE);

	return 0;
}

static void func_select_all_interface_for_msconfig(IUDEVICE* pdev, MSUSB_CONFIG_DESCRIPTOR* MsConfig)
{
	int inum;
	MSUSB_INTERFACE_DESCRIPTOR** MsInterfaces = MsConfig->MsInterfaces;
	BYTE  InterfaceNumber, AlternateSetting;
	UINT32 NumInterfaces = MsConfig->NumInterfaces;

	for (inum = 0; inum < NumInterfaces; inum++)
	{
		InterfaceNumber = MsInterfaces[inum]->InterfaceNumber;
		AlternateSetting = MsInterfaces[inum]->AlternateSetting;
		pdev->select_interface(pdev, InterfaceNumber, AlternateSetting);
	}
}

static int urb_select_configuration(URBDRC_CHANNEL_CALLBACK* callback,
									wStream* data, UINT32 MessageId,
									IUDEVMAN* udevman, UINT32 UsbDevice, int transferDir)
{
	MSUSB_CONFIG_DESCRIPTOR * MsConfig = NULL;
	IUDEVICE* pdev = NULL;
	UINT32 out_size, InterfaceId, RequestId, NumInterfaces, usbd_status = 0;
	BYTE ConfigurationDescriptorIsValid;
	wStream* out_data;
	int MsOutSize = 0;

	if ((transferDir == 0) || (Stream_GetRemainingLength(data) < 9))
	{
		WLog_ERR(TAG,  "urb_select_configuration: not support transfer out");
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));
	Stream_Read_UINT32(data, RequestId);
	Stream_Read_UINT8(data, ConfigurationDescriptorIsValid);
	Stream_Read_UINT32(data, NumInterfaces);

	/** if ConfigurationDescriptorIsValid is zero, then just do nothing.*/
	if (ConfigurationDescriptorIsValid)
	{
		/* parser data for struct config */
		MsConfig = msusb_msconfig_read(data, NumInterfaces);
		/* select config */
		pdev->select_configuration(pdev, MsConfig->bConfigurationValue);
		/* select all interface */
		func_select_all_interface_for_msconfig(pdev, MsConfig);
		/* complete configuration setup */
		MsConfig = pdev->complete_msconfig_setup(pdev, MsConfig);
	}

	if (MsConfig)
		MsOutSize = MsConfig->MsOutSize;

	if (MsOutSize > 0)
		out_size = 36 + MsOutSize;
	else
		out_size = 44;

	out_data = Stream_New(NULL, out_size);
	if (!out_data)
		return -1;

	Stream_Write_UINT32(out_data, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data, MessageId);		/** message id */
	Stream_Write_UINT32(out_data, URB_COMPLETION_NO_DATA);	/** function id */
	Stream_Write_UINT32(out_data, RequestId);	/** RequestId */

	if (MsOutSize > 0)
	{
		/** CbTsUrbResult */
		Stream_Write_UINT32(out_data, 8 + MsOutSize);
		/** TS_URB_RESULT_HEADER Size*/
		Stream_Write_UINT16(out_data, 8 + MsOutSize);
	}
	else
	{
		Stream_Write_UINT32(out_data, 16);
		Stream_Write_UINT16(out_data, 16);
	}

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data, URB_FUNCTION_SELECT_CONFIGURATION);
	Stream_Write_UINT32(out_data, usbd_status);	/** UsbdStatus */

	/** TS_URB_SELECT_CONFIGURATION_RESULT */
	if (MsOutSize > 0)
	{
		if (!msusb_msconfig_write(MsConfig, out_data))
		{
			Stream_Free(out_data, TRUE);
			return -1;
		}
	}
	else
	{
		Stream_Write_UINT32(out_data, 0);	/** ConfigurationHandle */
		Stream_Write_UINT32(out_data, NumInterfaces);	/** NumInterfaces */
	}
	Stream_Write_UINT32(out_data, 0);	/** HResult */
	Stream_Write_UINT32(out_data, 0);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(out_data),
								 Stream_Buffer(out_data), NULL);
	Stream_Free(out_data, TRUE);
	return 0;
}

static int urb_select_interface(URBDRC_CHANNEL_CALLBACK* callback, wStream* data,
								UINT32 MessageId, IUDEVMAN* udevman,
								UINT32 UsbDevice, int transferDir)
{
	MSUSB_CONFIG_DESCRIPTOR* MsConfig;
	MSUSB_INTERFACE_DESCRIPTOR* MsInterface;
	IUDEVICE* pdev;
	UINT32 InterfaceId, RequestId, ConfigurationHandle;
	UINT32 OutputBufferSize;
	BYTE InterfaceNumber;
	wStream* out_data;
	int interface_size;

	if (transferDir == 0)
	{
		WLog_ERR(TAG,  "urb_select_interface: not support transfer out");
		return -1;
	}

	if (Stream_GetRemainingLength(data) < 8)
		return ERROR_INVALID_DATA;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY << 30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data, RequestId);
	Stream_Read_UINT32(data, ConfigurationHandle);

	MsInterface = msusb_msinterface_read(data);
	if (!MsInterface)
		return -1;

	if (Stream_GetRemainingLength(data) < 4)
		return ERROR_INVALID_DATA;

	Stream_Read_UINT32(data, OutputBufferSize);

	pdev->select_interface(pdev, MsInterface->InterfaceNumber, MsInterface->AlternateSetting);

	/* replace device's MsInterface */
	MsConfig = pdev->get_MsConfig(pdev);
	InterfaceNumber = MsInterface->InterfaceNumber;
	msusb_msinterface_replace(MsConfig, InterfaceNumber, MsInterface);

	/* complete configuration setup */
	MsConfig = pdev->complete_msconfig_setup(pdev, MsConfig);
	MsInterface = MsConfig->MsInterfaces[InterfaceNumber];
	interface_size = 16 + (MsInterface->NumberOfPipes * 20);

	out_data = Stream_New(NULL, 36 + interface_size);
	if (!out_data)
		return -1;

	Stream_Write_UINT32(out_data, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data, MessageId);		/** message id */
	Stream_Write_UINT32(out_data, URB_COMPLETION_NO_DATA);	/** function id */
	Stream_Write_UINT32(out_data, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data, 8 + interface_size);	/** CbTsUrbResult */
	/** TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data, 8 + interface_size);	/** Size */
	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data, URB_FUNCTION_SELECT_INTERFACE);
	Stream_Write_UINT32(out_data, USBD_STATUS_SUCCESS);	/** UsbdStatus */

	/** TS_URB_SELECT_INTERFACE_RESULT */
	if (!msusb_msinterface_write(MsInterface, out_data))
	{
		Stream_Free(out_data, TRUE);
		return -1;
	}

	Stream_Write_UINT32(out_data, 0);	/** HResult */
	Stream_Write_UINT32(out_data, OutputBufferSize);	/** OutputBufferSize */
	Stream_SealLength(out_data);

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(out_data),
								 Stream_Pointer(out_data), NULL);

	Stream_Free(out_data, TRUE);

	return 0;
}

static int urb_control_transfer(URBDRC_CHANNEL_CALLBACK* callback, wStream* data,
								UINT32 MessageId, IUDEVMAN* udevman,
								UINT32 UsbDevice, int transferDir, int External)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, EndpointAddress, PipeHandle;
	UINT32 TransferFlags, OutputBufferSize, usbd_status, Timeout;
	BYTE bmRequestType, Request;
	UINT16 Value, Index, length;
	wStream* out_data;
	int ret;

	if (Stream_GetRemainingLength(data) < 24)
		return -1;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY << 30) | pdev->get_ReqCompletion(pdev));
	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, PipeHandle);
	Stream_Read_UINT32(data + 8, TransferFlags); /** TransferFlags */

	EndpointAddress = (PipeHandle & 0x000000ff);
	Timeout = 2000;

	switch (External)
	{
	case URB_CONTROL_TRANSFER_EXTERNAL:
		if (Stream_GetRemainingLength(data) < 16)
			return -1;

		Stream_Read_UINT32(data, Timeout); /** TransferFlags */
		break;
	case URB_CONTROL_TRANSFER_NONEXTERNAL:
		break;
	}

	/** SetupPacket 8 bytes */
	Stream_Read_UINT8(data, bmRequestType);
	Stream_Read_UINT8(data, Request);
	Stream_Read_UINT16(data, Value);
	Stream_Read_UINT16(data, Index);
	Stream_Read_UINT16(data, length);
	Stream_Read_UINT32(data, OutputBufferSize);

	if (length != OutputBufferSize)
	{
		WLog_ERR(TAG, "urb_control_transfer ERROR: buf != length");
		return -1;
	}

	out_size = 36 + OutputBufferSize;
	out_data = Stream_New(NULL, out_size);
	if (!out_data)
		return -1;

	Stream_Seek(out_data, 36);

	/** Get Buffer Data */
	if (transferDir == USBD_TRANSFER_DIRECTION_OUT)
		Stream_Copy(out_data, data, OutputBufferSize);

	/**  process URB_FUNCTION_CONTROL_TRANSFER */
	ret = pdev->control_transfer(
				pdev, RequestId, EndpointAddress, TransferFlags,
				bmRequestType,
				Request,
				Value,
				Index,
				&usbd_status,
				out_data,
				Timeout);

	if (ret < 0){
		WLog_DBG(TAG, "control_transfer: error num %d!!", ret);
		OutputBufferSize = 0;
	}

	/** send data */
	offset = 36;
	if (transferDir == USBD_TRANSFER_DIRECTION_IN)
		out_size = offset + OutputBufferSize;
	else
		out_size = offset;

	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);		/** message id */

	if(transferDir == USBD_TRANSFER_DIRECTION_IN && OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 0x00000008); 	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_CONTROL_TRANSFER);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);

	free(out_data);

	return 0;
}

static int urb_bulk_or_interrupt_transfer(URBDRC_CHANNEL_CALLBACK* callback,
										  wStream* data, UINT32 MessageId,
										  IUDEVMAN* udevman, UINT32 UsbDevice, int transferDir)
{
	int offset;
	BYTE* Buffer;
	IUDEVICE* pdev;
	BYTE* out_data;
	UINT32 out_size, RequestId, InterfaceId, EndpointAddress, PipeHandle;
	UINT32 TransferFlags, OutputBufferSize, usbd_status = 0;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY << 30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, PipeHandle);
	Stream_Read_UINT32(data + 8, TransferFlags);	/** TransferFlags */
	Stream_Read_UINT32(data + 12, OutputBufferSize);
	offset = 16;
	EndpointAddress = (PipeHandle & 0x000000ff);

	if (transferDir == USBD_TRANSFER_DIRECTION_OUT)
		out_size = 36;
	else
		out_size = 36 + OutputBufferSize;

	Buffer = NULL;
	out_data = (BYTE*) malloc(out_size);
	memset(out_data, 0, out_size);

	switch (transferDir)
	{
	case USBD_TRANSFER_DIRECTION_OUT:
		Buffer = data + offset;
		break;

	case USBD_TRANSFER_DIRECTION_IN:
		Buffer = out_data + 36;
		break;
	}

	/**  process URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER */
	pdev->bulk_or_interrupt_transfer(
				pdev, RequestId, EndpointAddress,
				TransferFlags,
				&usbd_status,
				&OutputBufferSize,
				Buffer,
				10000);

	offset = 36;
	if (transferDir == USBD_TRANSFER_DIRECTION_IN)
		out_size = offset + OutputBufferSize;
	else
		out_size = offset;
	/** send data */
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */
	if(transferDir == USBD_TRANSFER_DIRECTION_IN && OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (pdev && !pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);

	free(out_data);

	return 0;
}


static int urb_isoch_transfer(URBDRC_CHANNEL_CALLBACK * callback,
							  wStream* data,
							  UINT32 MessageId,
							  IUDEVMAN * udevman,
							  UINT32 UsbDevice,
							  int transferDir)
{
	IUDEVICE * pdev;
	UINT32	RequestId, InterfaceId, EndpointAddress;
	UINT32	PipeHandle, TransferFlags, StartFrame, NumberOfPackets;
	UINT32	ErrorCount, OutputBufferSize, usbd_status = 0;
	UINT32	RequestField, noAck = 0;
	UINT32	out_size = 0;
	BYTE *	iso_buffer	= NULL;
	BYTE *	iso_packets	= NULL;
	BYTE *	out_data	= NULL;
	int	offset, nullBuffer = 0, iso_status;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	if (pdev->isSigToEnd(pdev))
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));
	Stream_Read_UINT32(data + 0, RequestField);
	RequestId			= RequestField & 0x7fffffff;
	noAck				= (RequestField & 0x80000000)>>31;
	Stream_Read_UINT32(data + 4, PipeHandle);
	EndpointAddress		= (PipeHandle & 0x000000ff);
	Stream_Read_UINT32(data + 8, TransferFlags); /** TransferFlags */
	Stream_Read_UINT32(data + 12, StartFrame); /** StartFrame */
	Stream_Read_UINT32(data + 16, NumberOfPackets); /** NumberOfPackets */
	Stream_Read_UINT32(data + 20, ErrorCount); /** ErrorCount */
	offset = 24 + (NumberOfPackets * 12);
	Stream_Read_UINT32(data + offset, OutputBufferSize);
	offset += 4;

	/** send data memory alloc */
	if (transferDir == USBD_TRANSFER_DIRECTION_OUT) {
		if (!noAck) {
			out_size = 48 + (NumberOfPackets * 12);
			out_data = (BYTE *) malloc(out_size);
			iso_packets = out_data + 40;
		}
	}
	else {
		out_size = 48 + OutputBufferSize + (NumberOfPackets * 12);
		out_data = (BYTE *) malloc(out_size);
		iso_packets = out_data + 40;
	}

	if (out_size)
		memset(out_data, 0, out_size);

	switch (transferDir)
	{
	case USBD_TRANSFER_DIRECTION_OUT:
		/** Get Buffer Data */
		//memcpy(iso_buffer, data + offset, OutputBufferSize);
		iso_buffer = data + offset;
		break;
	case USBD_TRANSFER_DIRECTION_IN:
		iso_buffer = out_data + 48 + (NumberOfPackets * 12);
		break;
	}

	WLog_DBG(TAG, "urb_isoch_transfer: EndpointAddress: 0x%x, "
				  "TransferFlags: 0x%x, " "StartFrame: 0x%x, "
				  "NumberOfPackets: 0x%x, " "OutputBufferSize: 0x%x "
				  "RequestId: 0x%x",
			 EndpointAddress, TransferFlags, StartFrame,
			 NumberOfPackets, OutputBufferSize, RequestId);

#if ISOCH_FIFO
	ISOCH_CALLBACK_QUEUE * isoch_queue = NULL;
	ISOCH_CALLBACK_DATA * isoch = NULL;
	if(!noAck)
	{
		isoch_queue = (ISOCH_CALLBACK_QUEUE *)pdev->get_isoch_queue(pdev);
		isoch = isoch_queue->register_data(isoch_queue, callback, pdev);
	}
#endif

	iso_status = pdev->isoch_transfer(
				pdev, RequestId, EndpointAddress,
				TransferFlags,
				noAck,
				&ErrorCount,
				&usbd_status,
				&StartFrame,
				NumberOfPackets,
				iso_packets,
				&OutputBufferSize,
				iso_buffer,
				2000);

	if(noAck)
	{
		free(out_data);
		return 0;
	}

	if (iso_status < 0)
		nullBuffer = 1;


	out_size = 48;
	if (nullBuffer)
		OutputBufferSize = 0;
	else
		out_size += OutputBufferSize + (NumberOfPackets * 12);
	/* fill the send data */
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */
	if(OutputBufferSize != 0 && !nullBuffer)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 20 + (NumberOfPackets * 12));	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 20 + (NumberOfPackets * 12));	/** Size */
	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_ISOCH_TRANSFER);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, StartFrame);	/** StartFrame */
	if (!nullBuffer)
	{
		/** NumberOfPackets */
		Stream_Write_UINT32(out_data + 32, NumberOfPackets);
		Stream_Write_UINT32(out_data + 36, ErrorCount);	/** ErrorCount */
		offset = 40 + (NumberOfPackets * 12);
	}
	else
	{
		Stream_Write_UINT32(out_data + 32, 0);	/** NumberOfPackets */
		Stream_Write_UINT32(out_data + 36, NumberOfPackets);	/** ErrorCount */
		offset = 40;
	}

	Stream_Write_UINT32(out_data + offset, 0);	/** HResult */
	Stream_Write_UINT32(out_data + offset + 4, OutputBufferSize);	/** OutputBufferSize */

#if ISOCH_FIFO
	if(!noAck){
		if (WaitForSingleObject(isoch_queue->isoch_loading, INFINITE) != WAIT_OBJECT_0)
			return -1;
		isoch->out_data = out_data;
		isoch->out_size = out_size;
		if (!ReleaseMutex(isoch_queue->isoch_loading))
			return -1;
	}
#else
	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);
#endif

	if (nullBuffer)
		return -1;

	return 0;
}

static int urb_control_descriptor_request(URBDRC_CHANNEL_CALLBACK* callback,
										  wStream* data,
										  UINT32 MessageId,
										  IUDEVMAN * udevman,
										  UINT32 UsbDevice,
										  BYTE func_recipient,
										  int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, InterfaceId, RequestId, OutputBufferSize, usbd_status;
	BYTE bmRequestType, desc_index, desc_type;
	UINT16 langId;
	BYTE* buffer;
	wStream* out_data;
	size_t size;
	int ret, offset;

	if (Stream_GetRemainingLength(data) < 12)
		return -1;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));
	Stream_Read_UINT32(data, RequestId);
	Stream_Read_UINT8(data, desc_index);
	Stream_Read_UINT8(data, desc_type);
	Stream_Read_UINT16(data, langId);
	Stream_Read_UINT32(data, OutputBufferSize);

	out_size = 36 + OutputBufferSize;
	out_data = Stream_New(NULL, out_size);
	if (!out_data)
		return -1;

	buffer = Stream_Pointer(out_data) + 36;

	bmRequestType = func_recipient;
	switch (transferDir)
	{
	case USBD_TRANSFER_DIRECTION_IN:
		bmRequestType |= 0x80;
		break;
	case USBD_TRANSFER_DIRECTION_OUT:
		bmRequestType |= 0x00;
		offset = 12;
		Stream_Write(out_data, buffer, OutputBufferSize);
		break;
	default:
		WLog_DBG(TAG, "get error transferDir");
		OutputBufferSize = 0;
		usbd_status = USBD_STATUS_STALL_PID;
		break;
	}

	/** process get usb device descriptor */

	ret = pdev->control_transfer(
				pdev, RequestId, 0, 0, bmRequestType,
				0x06, /* REQUEST_GET_DESCRIPTOR */
				(desc_type << 8) | desc_index,
				langId,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				1000);


	if (ret < 0) {
		WLog_DBG(TAG, "get_descriptor: error num %d", ret);
		OutputBufferSize = 0;
	}

	offset = 36;
	out_size = offset + OutputBufferSize;
	if (!Stream_EnsureCapacity(out_data, out_size))
	{
		Stream_Free(out_data, TRUE);
		return -1;

	}

	size = Stream_GetPosition(out_data);
	Stream_SetPointer(out_data, 0);
	Stream_Write_UINT32(out_data, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data, MessageId);	/** message id */
	Stream_Write_UINT32(out_data, URB_COMPLETION);	/** function id */
	Stream_Write_UINT32(out_data, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data, 0x0008);	/** Size */
	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data, URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE);
	Stream_Write_UINT32(out_data, usbd_status);	/** UsbdStatus */
	Stream_Write_UINT32(out_data, 0);	/** HResult */
	Stream_Write_UINT32(out_data, OutputBufferSize);	/** OutputBufferSize */

	Stream_SetPosition(out_data, size);
	Stream_SealLength(out_data);

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, Stream_Length(out_data),
								 Stream_Buffer(out_data), NULL);

	Stream_Free(out_data, TRUE);
	return 0;
}

static int urb_control_get_status_request(URBDRC_CHANNEL_CALLBACK * callback,
										  wStream* data,
										  UINT32 MessageId,
										  IUDEVMAN * udevman,
										  UINT32 UsbDevice,
										  BYTE func_recipient,
										  int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize, usbd_status;
	UINT16 Index;
	BYTE bmRequestType;
	BYTE* buffer;
	BYTE* out_data;
	int offset, ret;

	if (transferDir == 0){
		WLog_DBG(TAG, "urb_control_get_status_request: not support transfer out");
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT16(data + 4, Index); /** Index */
	Stream_Read_UINT32(data + 8, OutputBufferSize);

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	bmRequestType = func_recipient | 0x80;

	ret = pdev->control_transfer(
				pdev, RequestId, 0, 0, bmRequestType,
				0x00, /* REQUEST_GET_STATUS */
				0,
				Index,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				1000);

	if (ret < 0){
		WLog_DBG(TAG, "control_transfer: error num %d!!", ret);
		OutputBufferSize = 0;
		usbd_status = USBD_STATUS_STALL_PID;
	}
	else{
		usbd_status = USBD_STATUS_SUCCESS;
	}

	/** send data */
	offset = 36;
	if (transferDir == USBD_TRANSFER_DIRECTION_IN)
		out_size = offset + OutputBufferSize;
	else
		out_size = offset;

	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);		/** message id */

	if(transferDir == USBD_TRANSFER_DIRECTION_IN && OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);

	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId, include NoAck*/
	Stream_Write_UINT32(out_data + 16, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);		/** Size */
	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_VENDOR_DEVICE);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);

	free(out_data);

	return 0;
}

static int urb_control_vendor_or_class_request(URBDRC_CHANNEL_CALLBACK * callback,
											   wStream* data,
											   UINT32 MessageId,
											   IUDEVMAN * udevman,
											   UINT32 UsbDevice,
											   BYTE func_type,
											   BYTE func_recipient,
											   int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, TransferFlags, usbd_status;
	UINT32 OutputBufferSize;
	BYTE ReqTypeReservedBits, Request, bmRequestType;
	UINT16 Value, Index, Padding;
	BYTE* buffer;
	BYTE* out_data;
	int offset, ret;
	/** control by vendor command */

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, TransferFlags); /** TransferFlags */
	Stream_Read_UINT8(data + 8, ReqTypeReservedBits); /** ReqTypeReservedBids */
	Stream_Read_UINT8(data + 9, Request); /** Request */
	Stream_Read_UINT16(data + 10, Value); /** value */
	Stream_Read_UINT16(data + 12, Index); /** index */
	Stream_Read_UINT16(data + 14, Padding); /** Padding */
	Stream_Read_UINT32(data + 16, OutputBufferSize);
	offset = 20;

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	/** Get Buffer */
	if (transferDir == USBD_TRANSFER_DIRECTION_OUT)
		memcpy(buffer, data + offset, OutputBufferSize);

	/** vendor or class command */
	bmRequestType = func_type | func_recipient;

	if (TransferFlags & USBD_TRANSFER_DIRECTION)
		bmRequestType |= 0x80;

	WLog_DBG(TAG, "urb_control_vendor_or_class_request: "
				  "RequestId 0x%x TransferFlags: 0x%x ReqTypeReservedBits: 0x%x "
				  "Request:0x%x Value: 0x%x Index: 0x%x OutputBufferSize: 0x%x bmRequestType: 0x%x!!",
			 RequestId, TransferFlags, ReqTypeReservedBits, Request, Value,
			 Index, OutputBufferSize, bmRequestType);

	ret = pdev->control_transfer(
				pdev, RequestId, 0, 0, bmRequestType,
				Request,
				Value,
				Index,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				2000);

	if (ret < 0){
		WLog_DBG(TAG, "control_transfer: error num %d!!", ret);
		OutputBufferSize = 0;
		usbd_status = USBD_STATUS_STALL_PID;
	}
	else{
		usbd_status = USBD_STATUS_SUCCESS;
	}

	offset = 36;
	if (transferDir == USBD_TRANSFER_DIRECTION_IN)
		out_size = offset + OutputBufferSize;
	else
		out_size = offset;
	/** send data */
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */

	if(transferDir == USBD_TRANSFER_DIRECTION_IN && OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);

	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId, include NoAck*/
	Stream_Write_UINT32(out_data + 16, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);	/** Size */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_VENDOR_DEVICE);	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);

	free(out_data);
	return 0;
}



static int urb_os_feature_descriptor_request(URBDRC_CHANNEL_CALLBACK * callback,
											 wStream* data,
											 UINT32 MessageId,
											 IUDEVMAN * udevman,
											 UINT32 UsbDevice,
											 int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize, usbd_status;
	BYTE Recipient, InterfaceNumber, Ms_PageIndex;
	UINT16 Ms_featureDescIndex;
	BYTE* out_data;
	BYTE* buffer;
	int offset, ret;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT8(data + 4, Recipient); /** Recipient */
	Recipient = (Recipient & 0x1f); /* XXX: origin: Recipient && 0x1f !? */
	Stream_Read_UINT8(data + 5, InterfaceNumber); /** InterfaceNumber */
	Stream_Read_UINT8(data + 6, Ms_PageIndex); /** Ms_PageIndex */
	Stream_Read_UINT16(data + 7, Ms_featureDescIndex); /** Ms_featureDescIndex */
	Stream_Read_UINT32(data + 12, OutputBufferSize);
	offset = 16;

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	switch (transferDir)
	{
	case USBD_TRANSFER_DIRECTION_OUT:
		WLog_ERR(TAG,  "Function urb_os_feature_descriptor_request: OUT Unchecked");
		memcpy(buffer, data + offset, OutputBufferSize);
		break;
	case USBD_TRANSFER_DIRECTION_IN:
		break;
	}

	WLog_DBG(TAG, "Ms descriptor arg: Recipient:0x%x, "
				  "InterfaceNumber:0x%x, Ms_PageIndex:0x%x, "
				  "Ms_featureDescIndex:0x%x, OutputBufferSize:0x%x",
			 Recipient, InterfaceNumber, Ms_PageIndex,
			 Ms_featureDescIndex, OutputBufferSize);
	/** get ms string */
	ret = pdev->os_feature_descriptor_request(
				pdev, RequestId, Recipient,
				InterfaceNumber,
				Ms_PageIndex,
				Ms_featureDescIndex,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				1000);

	if (ret < 0)
		WLog_DBG(TAG, "os_feature_descriptor_request: error num %d", ret);

	offset = 36;
	out_size = offset + OutputBufferSize;
	/** send data */
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */
	if(OutputBufferSize!=0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);	/** function id */
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_GET_MS_FEATURE_DESCRIPTOR);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);

	free(out_data);

	return 0;
}

static int urb_pipe_request(URBDRC_CHANNEL_CALLBACK * callback,
							wStream* data,
							UINT32 MessageId,
							IUDEVMAN * udevman,
							UINT32 UsbDevice,
							int transferDir,
							int action)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, PipeHandle, EndpointAddress;
	UINT32 OutputBufferSize, usbd_status = 0;
	BYTE* out_data;
	int out_offset, ret;

	if (transferDir == 0){
		WLog_DBG(TAG, "urb_pipe_request: not support transfer out");
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, PipeHandle); /** PipeHandle */
	Stream_Read_UINT32(data + 8, OutputBufferSize);
	EndpointAddress = (PipeHandle & 0x000000ff);


	switch (action){
	case PIPE_CANCEL:
		WLog_DBG(TAG, "urb_pipe_request: PIPE_CANCEL 0x%x ", EndpointAddress);

		ret = pdev->control_pipe_request(
					pdev, RequestId, EndpointAddress,
					&usbd_status,
					PIPE_CANCEL);

		if (ret < 0) {
			WLog_DBG(TAG, "PIPE SET HALT: error num %d", ret);
		}


		break;
	case PIPE_RESET:
		WLog_DBG(TAG, "urb_pipe_request: PIPE_RESET ep 0x%x ", EndpointAddress);

		ret = pdev->control_pipe_request(
					pdev, RequestId, EndpointAddress,
					&usbd_status,
					PIPE_RESET);

		if (ret < 0)
			WLog_DBG(TAG, "PIPE RESET: error num %d!!", ret);

		break;
	default:
		WLog_DBG(TAG, "urb_pipe_request action: %d is not support!", action);
		break;
	}


	/** send data */
	out_offset = 36;
	out_size = out_offset + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */

	Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 0x00000008);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 0x0008);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_SYNC_RESET_PIPE_AND_CLEAR_STALL);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, 0);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);

	return 0;
}

static int urb_get_current_frame_number(URBDRC_CHANNEL_CALLBACK* callback,
										wStream* data,
										UINT32 MessageId,
										IUDEVMAN * udevman,
										UINT32 UsbDevice,
										int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize;
	UINT32 dummy_frames;
	BYTE* out_data;

	if (transferDir == 0){
		WLog_DBG(TAG, "urb_get_current_frame_number: not support transfer out");
		//exit(1);
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, OutputBufferSize);

	dummy_frames = GetTickCount();

	out_size = 40;
	out_data = (BYTE *) calloc(1, out_size);
	if (!out_data)
		return -1;

	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */

	Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 12);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 12);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_GET_CURRENT_FRAME_NUMBER);
	Stream_Write_UINT32(out_data + 24, USBD_STATUS_SUCCESS);	/** UsbdStatus */
	Stream_Write_UINT32(out_data + 28, dummy_frames);	/** FrameNumber */

	Stream_Write_UINT32(out_data + 32, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 36, 0);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);
	return 0;
}


/* Unused function for current server */
static int urb_control_get_configuration_request(URBDRC_CHANNEL_CALLBACK* callback,
												 wStream* data,
												 UINT32 MessageId,
												 IUDEVMAN * udevman,
												 UINT32 UsbDevice,
												 int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize, usbd_status;
	BYTE* buffer;
	BYTE* out_data;
	int ret, offset;

	if (transferDir == 0)
	{
		WLog_DBG(TAG, "urb_control_get_configuration_request:"
					  " not support transfer out");
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT32(data + 4, OutputBufferSize);

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	ret = pdev->control_transfer(
				pdev, RequestId, 0, 0, 0x80 | 0x00,
				0x08, /* REQUEST_GET_CONFIGURATION */
				0,
				0,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				1000);

	if (ret < 0){
		WLog_DBG(TAG, "control_transfer: error num %d", ret);
		OutputBufferSize = 0;
	}


	offset = 36;
	out_size = offset + OutputBufferSize;
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);	/** message id */

	if (OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 8);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 8);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_GET_CONFIGURATION);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);
	return 0;
}

/* Unused function for current server */
static int urb_control_get_interface_request(URBDRC_CHANNEL_CALLBACK* callback,
											 wStream* data,
											 UINT32 MessageId,
											 IUDEVMAN * udevman,
											 UINT32 UsbDevice,
											 int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize, usbd_status;
	UINT16 interface;
	BYTE* buffer;
	BYTE* out_data;
	int ret, offset;

	if (transferDir == 0){
		WLog_DBG(TAG, "urb_control_get_interface_request: not support transfer out");
		return -1;
	}

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT16(data + 4, interface);
	Stream_Read_UINT32(data + 8, OutputBufferSize);

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	ret = pdev->control_transfer(pdev, RequestId, 0, 0, 0x80 | 0x01,
								 0x0A, /* REQUEST_GET_INTERFACE */
								 0,
								 interface,
								 &usbd_status,
								 &OutputBufferSize,
								 buffer,
								 1000);

	if (ret < 0){
		WLog_DBG(TAG, "control_transfer: error num %d", ret);
		OutputBufferSize = 0;
	}

	offset = 36;
	out_size = offset + OutputBufferSize;
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);		/** message id */

	if (OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 8);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 8);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_GET_INTERFACE);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */

	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);
	return 0;
}

static int urb_control_feature_request(URBDRC_CHANNEL_CALLBACK * callback,
									   wStream* data,
									   UINT32 MessageId,
									   IUDEVMAN * udevman,
									   UINT32 UsbDevice,
									   BYTE func_recipient,
									   BYTE command,
									   int transferDir)
{
	IUDEVICE* pdev;
	UINT32 out_size, RequestId, InterfaceId, OutputBufferSize, usbd_status;
	UINT16 FeatureSelector, Index;
	BYTE bmRequestType, bmRequest;
	BYTE* buffer;
	BYTE* out_data;
	int ret, offset;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);

	if (pdev == NULL)
		return 0;

	InterfaceId = ((STREAM_ID_PROXY<<30) | pdev->get_ReqCompletion(pdev));

	Stream_Read_UINT32(data + 0, RequestId);
	Stream_Read_UINT16(data + 4, FeatureSelector);
	Stream_Read_UINT16(data + 6, Index);
	Stream_Read_UINT32(data + 8, OutputBufferSize);
	offset = 12;

	out_size = 36 + OutputBufferSize;
	out_data = (BYTE *) malloc(out_size);
	memset(out_data, 0, out_size);

	buffer = out_data + 36;

	bmRequestType = func_recipient;
	switch (transferDir)
	{
	case USBD_TRANSFER_DIRECTION_OUT:
		WLog_ERR(TAG,  "Function urb_control_feature_request: OUT Unchecked");
		memcpy(buffer, data + offset, OutputBufferSize);
		bmRequestType |= 0x00;
		break;
	case USBD_TRANSFER_DIRECTION_IN:
		bmRequestType |= 0x80;
		break;
	}

	switch (command)
	{
	case URB_SET_FEATURE:
		bmRequest = 0x03; /* REQUEST_SET_FEATURE */
		break;
	case URB_CLEAR_FEATURE:
		bmRequest = 0x01; /* REQUEST_CLEAR_FEATURE */
		break;
	default:
		WLog_ERR(TAG,  "urb_control_feature_request: Error Command %x", command);
		free(out_data);
		return -1;
	}

	ret = pdev->control_transfer(
				pdev, RequestId, 0, 0, bmRequestType, bmRequest,
				FeatureSelector,
				Index,
				&usbd_status,
				&OutputBufferSize,
				buffer,
				1000);

	if (ret < 0){
		WLog_DBG(TAG, "feature control transfer: error num %d", ret);
		OutputBufferSize = 0;
	}

	offset = 36;
	out_size = offset + OutputBufferSize;
	Stream_Write_UINT32(out_data + 0, InterfaceId);	/** interface */
	Stream_Write_UINT32(out_data + 4, MessageId);		/** message id */

	if (OutputBufferSize != 0)
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION);
	else
		Stream_Write_UINT32(out_data + 8, URB_COMPLETION_NO_DATA);
	Stream_Write_UINT32(out_data + 12, RequestId);	/** RequestId */
	Stream_Write_UINT32(out_data + 16, 8);	/** CbTsUrbResult */
	/** TsUrbResult TS_URB_RESULT_HEADER */
	Stream_Write_UINT16(out_data + 20, 8);	/** Size */

	/** Padding, MUST be ignored upon receipt */
	Stream_Write_UINT16(out_data + 22, URB_FUNCTION_GET_INTERFACE);
	Stream_Write_UINT32(out_data + 24, usbd_status);	/** UsbdStatus */

	Stream_Write_UINT32(out_data + 28, 0);	/** HResult */
	Stream_Write_UINT32(out_data + 32, OutputBufferSize);	/** OutputBufferSize */


	if (!pdev->isSigToEnd(pdev))
		callback->channel->Write(callback->channel, out_size, out_data, NULL);
	free(out_data);
	return 0;
}

static int urbdrc_process_transfer_request(URBDRC_CHANNEL_CALLBACK * callback,
										   wStream* data,
										   UINT32 MessageId,
										   IUDEVMAN * udevman,
										   UINT32 UsbDevice,
										   int transferDir)
{
	IUDEVICE *	pdev;
	UINT32		CbTsUrb;
	UINT16		Size;
	UINT16		URB_Function;
	UINT32		OutputBufferSize;
	int			error = 0;
	size_t		pos;

	if (Stream_GetRemainingLength(data) < 12)
		return -1;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL)
		return 0;
	Stream_Read_UINT32(data, CbTsUrb);	/** CbTsUrb */
	Stream_Read_UINT16(data, Size);	/** size */
	Stream_Read_UINT16(data, URB_Function);
	if (Stream_GetRemainingLength(data) < 4 + CbTsUrb)
		return -1;
	pos = Stream_GetPosition(data);
	Stream_Seek(data, CbTsUrb);
	Stream_Read_UINT32(data, OutputBufferSize);

	Stream_SetPosition(data, pos);
	switch (URB_Function)
	{
	case URB_FUNCTION_SELECT_CONFIGURATION:			/** 0x0000 */
		error = urb_select_configuration(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_SELECT_INTERFACE:				/** 0x0001 */
		error = urb_select_interface(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_ABORT_PIPE:					/** 0x0002  */
		error = urb_pipe_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					PIPE_CANCEL);
		break;
	case URB_FUNCTION_TAKE_FRAME_LENGTH_CONTROL:	/** 0x0003  */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_TAKE_FRAME_LENGTH_CONTROL");
		error = -1;  /** This URB function is obsolete in Windows 2000
							 * and later operating systems
							 * and is not supported by Microsoft. */
		break;
	case URB_FUNCTION_RELEASE_FRAME_LENGTH_CONTROL:	/** 0x0004 */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RELEASE_FRAME_LENGTH_CONTROL");
		error = -1;  /** This URB function is obsolete in Windows 2000
							 * and later operating systems
							 * and is not supported by Microsoft. */
		break;
	case URB_FUNCTION_GET_FRAME_LENGTH:				/** 0x0005 */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_GET_FRAME_LENGTH");
		error = -1;  /** This URB function is obsolete in Windows 2000
							 * and later operating systems
							 * and is not supported by Microsoft. */
		break;
	case URB_FUNCTION_SET_FRAME_LENGTH:				/** 0x0006 */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_SET_FRAME_LENGTH");
		error = -1;  /** This URB function is obsolete in Windows 2000
							 * and later operating systems
							 * and is not supported by Microsoft. */
		break;
	case URB_FUNCTION_GET_CURRENT_FRAME_NUMBER:		/** 0x0007 */
		error = urb_get_current_frame_number(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_CONTROL_TRANSFER:				/** 0x0008 */
		error = urb_control_transfer(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					URB_CONTROL_TRANSFER_NONEXTERNAL);
		break;
	case URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER:	/** 0x0009 */
		error = urb_bulk_or_interrupt_transfer(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_ISOCH_TRANSFER:				/** 0x000A */
		error = urb_isoch_transfer(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE:	/** 0x000B */
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x00,
					transferDir);
		break;
	case URB_FUNCTION_SET_DESCRIPTOR_TO_DEVICE:		/** 0x000C */
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x00,
					transferDir);
		break;
	case URB_FUNCTION_SET_FEATURE_TO_DEVICE:		/** 0x000D */
		error = urb_control_feature_request(callback,
											data,
											MessageId,
											udevman,
											UsbDevice,
											0x00,
											URB_SET_FEATURE,
											transferDir);
		break;
	case URB_FUNCTION_SET_FEATURE_TO_INTERFACE:		/** 0x000E */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x01,
					URB_SET_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_SET_FEATURE_TO_ENDPOINT:		/** 0x000F */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x02,
					URB_SET_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_CLEAR_FEATURE_TO_DEVICE:		/** 0x0010 */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x00,
					URB_CLEAR_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_CLEAR_FEATURE_TO_INTERFACE:	/** 0x0011 */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x01,
					URB_CLEAR_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_CLEAR_FEATURE_TO_ENDPOINT:	/** 0x0012 */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x02,
					URB_CLEAR_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_GET_STATUS_FROM_DEVICE:		/** 0x0013 */
		error = urb_control_get_status_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x00,
					transferDir);
		break;
	case URB_FUNCTION_GET_STATUS_FROM_INTERFACE:	/** 0x0014 */
		error = urb_control_get_status_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x01,
					transferDir);
		break;
	case URB_FUNCTION_GET_STATUS_FROM_ENDPOINT:		/** 0x0015 */
		error = urb_control_get_status_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x02,
					transferDir);
		break;
	case URB_FUNCTION_RESERVED_0X0016:				/** 0x0016 */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVED_0X0016");
		error = -1;
		break;
	case URB_FUNCTION_VENDOR_DEVICE:				/** 0x0017 */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x02 << 5), /* vendor type */
					0x00,
					transferDir);
		break;
	case URB_FUNCTION_VENDOR_INTERFACE:				/** 0x0018 */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x02 << 5), /* vendor type */
					0x01,
					transferDir);
		break;
	case URB_FUNCTION_VENDOR_ENDPOINT:				/** 0x0019 */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x02 << 5), /* vendor type */
					0x02,
					transferDir);
		break;
	case URB_FUNCTION_CLASS_DEVICE:					/** 0x001A */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x01 << 5), /* class type */
					0x00,
					transferDir);
		break;
	case URB_FUNCTION_CLASS_INTERFACE:				/** 0x001B */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x01 << 5), /* class type */
					0x01,
					transferDir);
		break;
	case URB_FUNCTION_CLASS_ENDPOINT:				/** 0x001C */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x01 << 5), /* class type */
					0x02,
					transferDir);
		break;
	case URB_FUNCTION_RESERVE_0X001D:				/** 0x001D */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X001D");
		error = -1;
		break;
	case URB_FUNCTION_SYNC_RESET_PIPE_AND_CLEAR_STALL: /** 0x001E */
		error = urb_pipe_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					PIPE_RESET);
		break;
	case URB_FUNCTION_CLASS_OTHER:					/** 0x001F */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x01 << 5), /* class type */
					0x03,
					transferDir);
		break;
	case URB_FUNCTION_VENDOR_OTHER:					/** 0x0020 */
		error = urb_control_vendor_or_class_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					(0x02 << 5), /* vendor type */
					0x03,
					transferDir);
		break;
	case URB_FUNCTION_GET_STATUS_FROM_OTHER:		/** 0x0021 */
		error = urb_control_get_status_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x03,
					transferDir);
		break;
	case URB_FUNCTION_CLEAR_FEATURE_TO_OTHER:		/** 0x0022 */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x03,
					URB_CLEAR_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_SET_FEATURE_TO_OTHER:			/** 0x0023 */
		error = urb_control_feature_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x03,
					URB_SET_FEATURE,
					transferDir);
		break;
	case URB_FUNCTION_GET_DESCRIPTOR_FROM_ENDPOINT:	/** 0x0024 */
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x02,
					transferDir);
		break;
	case URB_FUNCTION_SET_DESCRIPTOR_TO_ENDPOINT:	/** 0x0025 */
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x02,
					transferDir);
		break;
	case URB_FUNCTION_GET_CONFIGURATION:			/** 0x0026 */
		error =  urb_control_get_configuration_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_GET_INTERFACE:				/** 0x0027 */
		error =  urb_control_get_interface_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE:	/** 0x0028 */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE");
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x01,
					transferDir);
		break;
	case URB_FUNCTION_SET_DESCRIPTOR_TO_INTERFACE:	/** 0x0029 */
		error = urb_control_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					0x01,
					transferDir);
		break;
	case URB_FUNCTION_GET_MS_FEATURE_DESCRIPTOR:	/** 0x002A */
		error = urb_os_feature_descriptor_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir);
		break;
	case URB_FUNCTION_RESERVE_0X002B:				/** 0x002B */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X002B");
		error = -1;
		break;
	case URB_FUNCTION_RESERVE_0X002C:				/** 0x002C */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X002C");
		error = -1;
		break;
	case URB_FUNCTION_RESERVE_0X002D:				/** 0x002D */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X002D");
		error = -1;
		break;
	case URB_FUNCTION_RESERVE_0X002E:				/** 0x002E */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X002E");
		error = -1;
		break;
	case URB_FUNCTION_RESERVE_0X002F:				/** 0x002F */
		WLog_DBG(TAG, "URB_Func: URB_FUNCTION_RESERVE_0X002F");
		error = -1;
		break;
		/** USB 2.0 calls start at 0x0030 */
	case URB_FUNCTION_SYNC_RESET_PIPE:				/** 0x0030 */
		error = urb_pipe_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					PIPE_RESET);
		error = -9;  /** function not support */
		break;
	case URB_FUNCTION_SYNC_CLEAR_STALL:				/** 0x0031 */
		error = urb_pipe_request(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					PIPE_RESET);
		error = -9;
		break;
	case URB_FUNCTION_CONTROL_TRANSFER_EX:			/** 0x0032 */
		error = urb_control_transfer(
					callback, data,
					MessageId,
					udevman,
					UsbDevice,
					transferDir,
					URB_CONTROL_TRANSFER_EXTERNAL);
		break;
	default:
		WLog_DBG(TAG, "URB_Func: %x is not found!", URB_Function);
		break;
	}

	return error;
}

void* urbdrc_process_udev_data_transfer(void* arg)
{
	TRANSFER_DATA*  transfer_data = (TRANSFER_DATA*) arg;
	URBDRC_CHANNEL_CALLBACK * callback = transfer_data->callback;
	wStream*	pBuffer		= transfer_data->pBuffer;
	UINT32		UsbDevice	= transfer_data->UsbDevice;
	IUDEVMAN *	udevman		= transfer_data->udevman;
	UINT32		MessageId;
	UINT32		FunctionId;
	IUDEVICE*   pdev;
	int error = 0;

	pdev = udevman->get_udevice_by_UsbDevice(udevman, UsbDevice);
	if (pdev == NULL || pdev->isSigToEnd(pdev) || (Stream_GetRemainingLength(pBuffer) < 8))
	{
		if (transfer_data)
		{
			if (transfer_data->pBuffer)
				Stream_Free(transfer_data->pBuffer, TRUE);
			free(transfer_data);
		}
		return 0;
	}

	pdev->push_action(pdev);

	/* USB kernel driver detach!! */
	pdev->detach_kernel_driver(pdev);

	Stream_Read_UINT32(pBuffer, MessageId);
	Stream_Read_UINT32(pBuffer, FunctionId);
	switch (FunctionId)
	{
	case CANCEL_REQUEST:
		error = urbdrc_process_cancel_request(
					pBuffer,
					udevman,
					UsbDevice);
		break;
	case REGISTER_REQUEST_CALLBACK:
		error = urbdrc_process_register_request_callback(
					callback,
					pBuffer,
					udevman,
					UsbDevice);
		break;
	case IO_CONTROL:
		error = urbdrc_process_io_control(
					callback,
					pBuffer,
					MessageId,
					udevman, UsbDevice);
		break;
	case INTERNAL_IO_CONTROL:
		error = urbdrc_process_internal_io_control(
					callback,
					pBuffer,
					MessageId,
					udevman, UsbDevice);
		break;
	case QUERY_DEVICE_TEXT:
		error = urbdrc_process_query_device_text(
					callback,
					pBuffer,
					MessageId,
					udevman,
					UsbDevice);
		break;
	case TRANSFER_IN_REQUEST:
		error = urbdrc_process_transfer_request(
					callback,
					pBuffer,
					MessageId,
					udevman,
					UsbDevice,
					USBD_TRANSFER_DIRECTION_IN);
		break;
	case TRANSFER_OUT_REQUEST:
		error = urbdrc_process_transfer_request(
					callback,
					pBuffer,
					MessageId,
					udevman,
					UsbDevice,
					USBD_TRANSFER_DIRECTION_OUT);
		break;
	case RETRACT_DEVICE:
		error = urbdrc_process_retract_device_request(
					pBuffer,
					udevman,
					UsbDevice);
		break;
	default:
		WLog_DBG(TAG, "urbdrc_process_udev_data_transfer:"
					  " unknown FunctionId 0x%X", FunctionId);
		error = -1;
		break;
	}

	if (transfer_data)
	{
		Stream_Free(transfer_data->pBuffer, TRUE);
		free(transfer_data);
	}

	if (pdev)
	{
#if ISOCH_FIFO
		/* check isochronous fds */
		func_check_isochronous_fds(pdev);
#endif
		/* close this channel, if device is not found. */
		pdev->complete_action(pdev);
	}
	else
	{
		udevman->push_urb(udevman);
		return error;
	}

	udevman->push_urb(udevman);
	return error;
}
