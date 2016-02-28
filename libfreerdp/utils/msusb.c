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
 *	 http://www.apache.org/licenses/LICENSE-2.0
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

#include <freerdp/log.h>
#include <freerdp/utils/msusb.h>

#define TAG FREERDP_TAG("utils")

#ifdef WITH_DEBUG_MSUSB
#define DEBUG_MSUSB(fmt, ...) WLog_DBG(TAG, fmt, ## __VA_ARGS__)
#else
#define DEBUG_MSUSB(fmt, ...) do { } while (0)
#endif


static MSUSB_PIPE_DESCRIPTOR* msusb_mspipe_new()
{
	return (MSUSB_PIPE_DESCRIPTOR*) calloc(1, sizeof(MSUSB_PIPE_DESCRIPTOR));
}

static void msusb_mspipes_free(MSUSB_PIPE_DESCRIPTOR** MsPipes, UINT32 NumberOfPipes)
{
	int pnum = 0;

	if (MsPipes)
	{
		for (pnum = 0; pnum < NumberOfPipes && MsPipes[pnum]; pnum++)
		{
				free(MsPipes[pnum]);
		}
		free(MsPipes);
	}
}

void msusb_mspipes_replace(MSUSB_INTERFACE_DESCRIPTOR* MsInterface, MSUSB_PIPE_DESCRIPTOR** NewMsPipes, UINT32 NewNumberOfPipes)
{
	/* free orignal MsPipes */
	msusb_mspipes_free(MsInterface->MsPipes, MsInterface->NumberOfPipes);
	/* And replace it */
	MsInterface->MsPipes = NewMsPipes;
	MsInterface->NumberOfPipes = NewNumberOfPipes;
}

static MSUSB_PIPE_DESCRIPTOR** msusb_mspipes_read(wStream* data, UINT32 NumberOfPipes)
{
	int pnum;
	MSUSB_PIPE_DESCRIPTOR** MsPipes;

	if (Stream_GetRemainingLength(data) < 12 * NumberOfPipes)
		return NULL;

	MsPipes = (MSUSB_PIPE_DESCRIPTOR**) calloc(NumberOfPipes, sizeof(MSUSB_PIPE_DESCRIPTOR*));
	if (!MsPipes)
		return NULL;
	

	for (pnum = 0; pnum < NumberOfPipes; pnum++)
	{
		MSUSB_PIPE_DESCRIPTOR *MsPipe = msusb_mspipe_new();
		if (!MsPipe)
			goto out_error;
		
		Stream_Read_UINT16(data, MsPipe->MaximumPacketSize);
		Stream_Seek(data, 2);
		Stream_Read_UINT32(data, MsPipe->MaximumTransferSize);
		Stream_Read_UINT32(data, MsPipe->PipeFlags);

/* Already set to zero by memset
		MsPipe->PipeHandle	   = 0;
		MsPipe->bEndpointAddress = 0;
		MsPipe->bInterval		= 0;
		MsPipe->PipeType		 = 0;
		MsPipe->InitCompleted	= 0;
*/
		
		MsPipes[pnum] = MsPipe;
	}
	
	return MsPipes;

out_error:
	for (pnum = 0; pnum < NumberOfPipes; pnum++)
		free(MsPipes[pnum]);

	free(MsPipes);
	return NULL;
}

static MSUSB_INTERFACE_DESCRIPTOR* msusb_msinterface_new(void)
{
	return (MSUSB_INTERFACE_DESCRIPTOR*) calloc(1, sizeof(MSUSB_INTERFACE_DESCRIPTOR));
}

static void msusb_msinterface_free(MSUSB_INTERFACE_DESCRIPTOR* MsInterface)
{
	if (MsInterface)
	{
		msusb_mspipes_free(MsInterface->MsPipes, MsInterface->NumberOfPipes);
		MsInterface->MsPipes = NULL;
		free(MsInterface);
	}
}

static void msusb_msinterface_free_list(MSUSB_INTERFACE_DESCRIPTOR** MsInterfaces, UINT32 NumInterfaces)
{
	int inum = 0;

	if (MsInterfaces)
	{
		for (inum = 0; inum < NumInterfaces; inum++)
		{
			msusb_msinterface_free(MsInterfaces[inum]);
		}

		free(MsInterfaces);
	}
}

void msusb_msinterface_replace(MSUSB_CONFIG_DESCRIPTOR* MsConfig, BYTE InterfaceNumber, MSUSB_INTERFACE_DESCRIPTOR* NewMsInterface)
{
	msusb_msinterface_free(MsConfig->MsInterfaces[InterfaceNumber]);
	MsConfig->MsInterfaces[InterfaceNumber] = NewMsInterface;	
}

MSUSB_INTERFACE_DESCRIPTOR* msusb_msinterface_read(wStream* data)
{
	MSUSB_INTERFACE_DESCRIPTOR* MsInterface;
	
	if (Stream_GetRemainingLength(data) < 12)
	MsInterface = msusb_msinterface_new();
	if (!MsInterface)
		return NULL;
	
	Stream_Read_UINT16(data, MsInterface->Length);
	Stream_Read_UINT16(data, MsInterface->NumberOfPipesExpected);
	Stream_Read_UINT8(data, MsInterface->InterfaceNumber);
	Stream_Read_UINT8(data, MsInterface->AlternateSetting);
	Stream_Seek(data, 2);
	Stream_Read_UINT32(data, MsInterface->NumberOfPipes);

	MsInterface->InterfaceHandle = 0;
	MsInterface->bInterfaceClass = 0;
	MsInterface->bInterfaceSubClass = 0;
	MsInterface->bInterfaceProtocol = 0;
	MsInterface->InitCompleted = 0;
	MsInterface->MsPipes = NULL;
	
	if (MsInterface->NumberOfPipes > 0)
	{
		MsInterface->MsPipes = 
			msusb_mspipes_read(data, MsInterface->NumberOfPipes);
		if (!MsInterface->MsPipes)
			goto out_error;
	}
	
	return MsInterface;

out_error:
	msusb_msinterface_free(MsInterface);
	return NULL;
}

BOOL msusb_msinterface_write(MSUSB_INTERFACE_DESCRIPTOR* MsInterface, wStream* data)
{
	MSUSB_PIPE_DESCRIPTOR ** MsPipes;
	MSUSB_PIPE_DESCRIPTOR * MsPipe;
	int pnum = 0;
	
	if (!Stream_EnsureRemainingCapacity(data, 16 + 20 * MsInterface->NumberOfPipes))
		return FALSE;

	/* Length */
	Stream_Write_UINT16(data, MsInterface->Length);
	/* InterfaceNumber */
	Stream_Write_UINT8(data, MsInterface->InterfaceNumber);
	/* AlternateSetting */
	Stream_Write_UINT8(data, MsInterface->AlternateSetting);
	/* bInterfaceClass */
	Stream_Write_UINT8(data, MsInterface->bInterfaceClass);
	/* bInterfaceSubClass */
	Stream_Write_UINT8(data, MsInterface->bInterfaceSubClass);
	/* bInterfaceProtocol */
	Stream_Write_UINT8(data, MsInterface->bInterfaceProtocol);
	/* Padding */ 
	Stream_Write_UINT8(data, 0);
	/* InterfaceHandle */
	Stream_Write_UINT32(data, MsInterface->InterfaceHandle);
	/* NumberOfPipes */
	Stream_Write_UINT32(data, MsInterface->NumberOfPipes);

	/* Pipes */
	MsPipes = MsInterface->MsPipes;
	for(pnum = 0; pnum < MsInterface->NumberOfPipes; pnum++)
	{
		MsPipe = MsPipes[pnum];
		/* MaximumPacketSize */
		Stream_Write_UINT16(data, MsPipe->MaximumPacketSize);
		/* EndpointAddress */
		Stream_Write_UINT8(data, MsPipe->bEndpointAddress);
		/* Interval */
		Stream_Write_UINT8(data, MsPipe->bInterval);
		/* PipeType */
		Stream_Write_UINT32(data, MsPipe->PipeType);
		/* PipeHandle */
		Stream_Write_UINT32(data, MsPipe->PipeHandle);
		/* MaximumTransferSize */
		Stream_Write_UINT32(data, MsPipe->MaximumTransferSize);
		/* PipeFlags */
		Stream_Write_UINT32(data, MsPipe->PipeFlags);
	}

	return TRUE;
}

static MSUSB_INTERFACE_DESCRIPTOR** msusb_msinterface_read_list(wStream* data, UINT32 NumInterfaces)
{
	int inum;
	MSUSB_INTERFACE_DESCRIPTOR** MsInterfaces;
	
	MsInterfaces = (MSUSB_INTERFACE_DESCRIPTOR**) calloc(NumInterfaces, sizeof(MSUSB_INTERFACE_DESCRIPTOR*));
	if (!MsInterfaces)
		return NULL;
	
	for (inum = 0; inum < NumInterfaces; inum++)
	{
		MsInterfaces[inum] = msusb_msinterface_read(data);
		if (!MsInterfaces[inum])
		{
			while(inum > 0)
				msusb_msinterface_free(MsInterfaces[inum--]);
			free(MsInterfaces);
			return NULL;
		}
	}   
	
	return MsInterfaces;
}

BOOL msusb_msconfig_write(MSUSB_CONFIG_DESCRIPTOR* MsConfg, wStream* data)
{
	int inum = 0;
	MSUSB_INTERFACE_DESCRIPTOR** MsInterfaces;
	MSUSB_INTERFACE_DESCRIPTOR* MsInterface;
	
	if (!Stream_EnsureRemainingCapacity(data, 8))
		return FALSE;

	/* ConfigurationHandle*/
	Stream_Write_UINT32(data, MsConfg->ConfigurationHandle);

	/* NumInterfaces*/
	Stream_Write_UINT32(data, MsConfg->NumInterfaces);

	/* Interfaces */
	MsInterfaces = MsConfg->MsInterfaces;

	for(inum = 0; inum < MsConfg->NumInterfaces; inum++)
	{
		MsInterface = MsInterfaces[inum];
		if (!msusb_msinterface_write(MsInterface, data))
		{
			while(inum > 0)
				msusb_msinterface_free(MsInterfaces[inum--]);

			return FALSE;
		}
	}
	
	return TRUE;
}

MSUSB_CONFIG_DESCRIPTOR* msusb_msconfig_new(void)
{
	return (MSUSB_CONFIG_DESCRIPTOR*) calloc(1, sizeof(MSUSB_CONFIG_DESCRIPTOR));
}

void msusb_msconfig_free(MSUSB_CONFIG_DESCRIPTOR* MsConfig)
{
	if (MsConfig)
	{
		msusb_msinterface_free_list(MsConfig->MsInterfaces, MsConfig->NumInterfaces);
		MsConfig->MsInterfaces = NULL;
		free(MsConfig);
	}
}

MSUSB_CONFIG_DESCRIPTOR* msusb_msconfig_read(wStream* data, UINT32 NumInterfaces)
{
	MSUSB_CONFIG_DESCRIPTOR* MsConfig;
	BYTE lenConfiguration, typeConfiguration;
	
	if (!Stream_GetRemainingLength(data) < NumInterfaces * 2 + 5)
		return NULL;

	MsConfig = msusb_msconfig_new();
	if (!MsConfig)
		return NULL;

	Stream_Seek(data, NumInterfaces * 2);

	Stream_Read_UINT8(data, lenConfiguration);
	Stream_Read_UINT8(data, typeConfiguration);

	if (lenConfiguration != 0x9 || typeConfiguration != 0x2)
	{
		DEBUG_MSUSB("%s: len and type must be 0x9 and 0x2 , but it is 0x%x and 0x%x",
			lenConfiguration, typeConfiguration);
	}

	Stream_Read_UINT16(data, MsConfig->wTotalLength);
	Stream_Read_UINT8(data, MsConfig->bConfigurationValue);

	MsConfig->NumInterfaces	= NumInterfaces;
	MsConfig->ConfigurationHandle = 0;
	MsConfig->InitCompleted = 0;
	MsConfig->MsOutSize = 0;
	MsConfig->MsInterfaces = NULL;
	
	if (NumInterfaces > 0)
	{
		MsConfig->MsInterfaces = msusb_msinterface_read_list(data, NumInterfaces);
		if (!MsConfig->MsInterfaces)
		{
			msusb_msconfig_free(MsConfig);
			return NULL;
		}
	}
		
	return MsConfig;
}

void msusb_msconfig_dump(MSUSB_CONFIG_DESCRIPTOR* MsConfig)
{
	MSUSB_INTERFACE_DESCRIPTOR ** MsInterfaces;
	MSUSB_INTERFACE_DESCRIPTOR * MsInterface;
	MSUSB_PIPE_DESCRIPTOR ** MsPipes;
	MSUSB_PIPE_DESCRIPTOR * MsPipe;
	int inum = 0, pnum = 0;
	WLog_INFO(TAG,  "=================MsConfig:========================");
	WLog_INFO(TAG,  "wTotalLength:%d", MsConfig->wTotalLength);
	WLog_INFO(TAG,  "bConfigurationValue:%d", MsConfig->bConfigurationValue);
	WLog_INFO(TAG,  "ConfigurationHandle:0x%x", MsConfig->ConfigurationHandle);
	WLog_INFO(TAG,  "InitCompleted:%d", MsConfig->InitCompleted);
	WLog_INFO(TAG,  "MsOutSize:%d", MsConfig->MsOutSize);
	WLog_INFO(TAG,  "NumInterfaces:%d", MsConfig->NumInterfaces);
	MsInterfaces = MsConfig->MsInterfaces;
	for(inum = 0; inum < MsConfig->NumInterfaces; inum++)
	{
		MsInterface = MsInterfaces[inum];
		WLog_INFO(TAG,  "	Interfase: %d", MsInterface->InterfaceNumber);
		WLog_INFO(TAG,  "	Length: %d", MsInterface->Length);
		WLog_INFO(TAG,  "	NumberOfPipesExpected: %d", MsInterface->NumberOfPipesExpected);
		WLog_INFO(TAG,  "	AlternateSetting: %d", MsInterface->AlternateSetting);
		WLog_INFO(TAG,  "	NumberOfPipes: %d", MsInterface->NumberOfPipes);
		WLog_INFO(TAG,  "	InterfaceHandle: 0x%x", MsInterface->InterfaceHandle);
		WLog_INFO(TAG,  "	bInterfaceClass: 0x%x", MsInterface->bInterfaceClass);
		WLog_INFO(TAG,  "	bInterfaceSubClass: 0x%x", MsInterface->bInterfaceSubClass);
		WLog_INFO(TAG,  "	bInterfaceProtocol: 0x%x", MsInterface->bInterfaceProtocol);
		WLog_INFO(TAG,  "	InitCompleted: %d", MsInterface->InitCompleted);
		MsPipes = MsInterface->MsPipes;
		for (pnum = 0; pnum < MsInterface->NumberOfPipes; pnum++)
		{
			MsPipe = MsPipes[pnum];
			WLog_INFO(TAG,  "		Pipe: %d", pnum);
			WLog_INFO(TAG,  "		MaximumPacketSize: 0x%x", MsPipe->MaximumPacketSize);
			WLog_INFO(TAG,  "		MaximumTransferSize: 0x%x", MsPipe->MaximumTransferSize);
			WLog_INFO(TAG,  "		PipeFlags: 0x%x", MsPipe->PipeFlags);
			WLog_INFO(TAG,  "		PipeHandle: 0x%x", MsPipe->PipeHandle);
			WLog_INFO(TAG,  "		bEndpointAddress: 0x%x", MsPipe->bEndpointAddress);
			WLog_INFO(TAG,  "		bInterval: %d", MsPipe->bInterval);
			WLog_INFO(TAG,  "		PipeType: 0x%x", MsPipe->PipeType);
			WLog_INFO(TAG,  "		InitCompleted: %d", MsPipe->InitCompleted);
		}
	}
	WLog_INFO(TAG,  "==================================================");
}
