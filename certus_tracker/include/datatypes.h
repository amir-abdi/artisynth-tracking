#pragma once
/*****************************************************************
Name:			datatypes.h

Description:
	Data types used by the Optotrak sample programs.

*****************************************************************/

#define OPTO_APP_ERROR_CODE					3000
#define OPTOTRAK_REVISION_UNKNOWN_FLAG		0

typedef struct OptotrakSettingsStruct
{
	int nMarkers;
	int nThreshold;
	int nMinimumGain;
	int nStreamData;
	float fFrameFrequency;
	float fMarkerFrequency;
	float fDutyCycle;
	float fVoltage;
	float fCollectionTime;
	float fPreTriggerTime;
	unsigned int uFlags;
} OptotrakSettings;


typedef struct OptotrakStatusStruct
{
	int nSensors;
	int nOdaus;
	int nRigidBodies;
	OptotrakSettings dtSettings;
} OptotrakStatus;


typedef struct ApplicationDeviceInformationStruct
{
	char szName[ DEVICE_MAX_PROPERTY_STRING ];
	char szFiringSequence[ DEVICE_MAX_MARKERS ];
	char szSromFiringSequence[ DEVICE_MAX_MARKERS ];
	char szSromPartNumber[ DEVICE_MAX_PROPERTY_STRING ];
	char szSromManufacturer[ DEVICE_MAX_PROPERTY_STRING ];
	char szSerialNumber[ DEVICE_MAX_PROPERTY_STRING ];
	char szFeatureKey[ DEVICE_MAX_PROPERTY_STRING ];
	char szMkrPortEnc[ DEVICE_MAX_PROPERTY_STRING ];
	int nMarkersAvailable;
	int nMarkersToActivate;
	int n3020MarkersToActivate;
	int nStartMarkerPeriod;
	int n3020StartMarkerPeriod;
	int nPort;
	int nOrder;
	int nSubPort;
	int nVLEDs;
	int nSwitches;
	int nToolPorts;
	int nStatus;
	int nSromRigidMarkers;
	int nSromRigidPositions;
	int nSromNormalMarkers;
	int nSromNormals;
	int nSromToolType;
	int nScanMode;
	int nBumpSensor;
	int nWireless;
	int nBeeper;
	int nDeviceType;
	int nSmartMarkers;
	long lSromSerialNumber;
	long lStateFlags;
	long lTipId;
	boolean b3020Capability;
	boolean bHasROM;
} ApplicationDeviceInformation;
