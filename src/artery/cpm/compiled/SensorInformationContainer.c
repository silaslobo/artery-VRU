/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "SensorInformationContainer.h"

asn_TYPE_member_t asn_MBR_SensorInformationContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformationContainer, id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Identifier,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformationContainer, type),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SensorType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"type"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SensorInformationContainer, details),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SensorDetails,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"details"
		},
};
static const ber_tlv_tag_t asn_DEF_SensorInformationContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SensorInformationContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* type */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* details */
};
asn_SEQUENCE_specifics_t asn_SPC_SensorInformationContainer_specs_1 = {
	sizeof(struct SensorInformationContainer),
	offsetof(struct SensorInformationContainer, _asn_ctx),
	asn_MAP_SensorInformationContainer_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SensorInformationContainer = {
	"SensorInformationContainer",
	"SensorInformationContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_SensorInformationContainer_tags_1,
	sizeof(asn_DEF_SensorInformationContainer_tags_1)
		/sizeof(asn_DEF_SensorInformationContainer_tags_1[0]), /* 1 */
	asn_DEF_SensorInformationContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_SensorInformationContainer_tags_1)
		/sizeof(asn_DEF_SensorInformationContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_SensorInformationContainer_1,
	3,	/* Elements count */
	&asn_SPC_SensorInformationContainer_specs_1	/* Additional specs */
};

