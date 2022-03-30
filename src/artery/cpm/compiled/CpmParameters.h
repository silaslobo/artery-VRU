/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_CpmParameters_H_
#define	_CpmParameters_H_


#include "asn_application.h"

/* Including external dependencies */
#include "CpmManagementContainer.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct StationDataContainer;
struct ListOfSensorInformationContainer;
struct ListOfPerceivedObjectContainer;

/* CpmParameters */
typedef struct CpmParameters {
	CpmManagementContainer_t	 managementContainer;
	struct StationDataContainer	*stationDataContainer	/* OPTIONAL */;
	struct ListOfSensorInformationContainer	*sensorInformationContainer	/* OPTIONAL */;
	struct ListOfPerceivedObjectContainer	*perceivedObjectContainer	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CpmParameters_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CpmParameters;
extern asn_SEQUENCE_specifics_t asn_SPC_CpmParameters_specs_1;
extern asn_TYPE_member_t asn_MBR_CpmParameters_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "StationDataContainer.h"
#include "ListOfSensorInformationContainer.h"
#include "ListOfPerceivedObjectContainer.h"

#endif	/* _CpmParameters_H_ */
#include "asn_internal.h"
