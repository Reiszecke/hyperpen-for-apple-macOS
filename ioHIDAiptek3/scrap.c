/*
 *  scrap.c
 *  ioHIDFunctions
 *
 *  Created by Paar on 19.10.11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


//---------------------------------------------------------------------------
// InterruptReportCallbackFunction
//---------------------------------------------------------------------------
void InterruptReportCallbackFunction
(void *	 		target,
 IOReturn 		result,
 void * 			refcon,
 void * 			sender,
 uint32_t		 	bufferSize)
{
    HIDDataRef hidDataRef = (HIDDataRef)refcon;
    int index;
    
    if ( !hidDataRef )
        return;
	
    printf("Buffer = ");
	
    for ( index=0; index<bufferSize; index++)
        printf("%2.2x ", hidDataRef->buffer[index]);
	
    printf("\n");
	
}

/* Obtain a device interface structure (hidDeviceInterface). */
result = (*plugInInterface)->QueryInterface(plugInInterface, CFUUIDGetUUIDBytes(kIOHIDDeviceInterfaceID122), 
											(LPVOID *)&hidDeviceInterface);

/* Open the device. */
result = (*(hidDataRef->hidDeviceInterface))->open (hidDataRef->hidDeviceInterface, 0);

/* Find the HID elements for this device */
pass = FindHIDElements(hidDataRef);

/* Create an asynchronous event source for this device. */
result = (*(hidDataRef->hidDeviceInterface))->createAsyncEventSource(hidDataRef->hidDeviceInterface, &hidDataRef->eventSource);

/* Set the handler to call when the device sends a report. */
result = (*(hidDataRef->hidDeviceInterface))->setInterruptReportHandlerCallback(hidDataRef->hidDeviceInterface, 
									hidDataRef->buffer, sizeof(hidDataRef->buffer), &InterruptReportCallbackFunction, NULL, hidDataRef);

/* Add the asynchronous event source to the run loop. */
CFRunLoopAddSource(CFRunLoopGetCurrent(), hidDataRef->eventSource, kCFRunLoopDefaultMode);


