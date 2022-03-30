/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_IntersectionTopologyPosition_H_
#define	_IntersectionTopologyPosition_H_


#include "asn_application.h"

/* Including external dependencies */
#include "LaneID.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct OffsetPoint;

/* IntersectionTopologyPosition */
typedef struct IntersectionTopologyPosition {
	LaneID_t	*laneID	/* OPTIONAL */;
	struct OffsetPoint	*laneOffset	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntersectionTopologyPosition_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntersectionTopologyPosition;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "OffsetPoint.h"

#endif	/* _IntersectionTopologyPosition_H_ */
#include "asn_internal.h"