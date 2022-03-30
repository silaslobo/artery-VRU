/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_Distance_H_
#define	_Distance_H_


#include "asn_application.h"

/* Including external dependencies */
#include "DistanceValue.h"
#include "DistanceConfidence.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Distance */
typedef struct Distance {
	DistanceValue_t	 value;
	DistanceConfidence_t	 confidence;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Distance_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Distance;
extern asn_SEQUENCE_specifics_t asn_SPC_Distance_specs_1;
extern asn_TYPE_member_t asn_MBR_Distance_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Distance_H_ */
#include "asn_internal.h"
