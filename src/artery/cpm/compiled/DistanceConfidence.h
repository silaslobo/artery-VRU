/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_DistanceConfidence_H_
#define	_DistanceConfidence_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DistanceConfidence {
	DistanceConfidence_zeroPointZeroOneMeter	= 1,
	DistanceConfidence_oneMeter	= 100
} e_DistanceConfidence;

/* DistanceConfidence */
typedef long	 DistanceConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_DistanceConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_DistanceConfidence;
asn_struct_free_f DistanceConfidence_free;
asn_struct_print_f DistanceConfidence_print;
asn_constr_check_f DistanceConfidence_constraint;
ber_type_decoder_f DistanceConfidence_decode_ber;
der_type_encoder_f DistanceConfidence_encode_der;
xer_type_decoder_f DistanceConfidence_decode_xer;
xer_type_encoder_f DistanceConfidence_encode_xer;
oer_type_decoder_f DistanceConfidence_decode_oer;
oer_type_encoder_f DistanceConfidence_encode_oer;
per_type_decoder_f DistanceConfidence_decode_uper;
per_type_encoder_f DistanceConfidence_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _DistanceConfidence_H_ */
#include "asn_internal.h"
