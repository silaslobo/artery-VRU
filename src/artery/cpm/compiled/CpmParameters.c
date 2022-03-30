/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "CpmParameters.h"

asn_TYPE_member_t asn_MBR_CpmParameters_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CpmParameters, managementContainer),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CpmManagementContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"managementContainer"
		},
	{ ATF_POINTER, 3, offsetof(struct CpmParameters, stationDataContainer),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_StationDataContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"stationDataContainer"
		},
	{ ATF_POINTER, 2, offsetof(struct CpmParameters, sensorInformationContainer),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ListOfSensorInformationContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"sensorInformationContainer"
		},
	{ ATF_POINTER, 1, offsetof(struct CpmParameters, perceivedObjectContainer),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ListOfPerceivedObjectContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"perceivedObjectContainer"
		},
};
static const int asn_MAP_CpmParameters_oms_1[] = { 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_CpmParameters_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CpmParameters_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* managementContainer */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* stationDataContainer */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* sensorInformationContainer */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* perceivedObjectContainer */
};
asn_SEQUENCE_specifics_t asn_SPC_CpmParameters_specs_1 = {
	sizeof(struct CpmParameters),
	offsetof(struct CpmParameters, _asn_ctx),
	asn_MAP_CpmParameters_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_CpmParameters_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CpmParameters = {
	"CpmParameters",
	"CpmParameters",
	&asn_OP_SEQUENCE,
	asn_DEF_CpmParameters_tags_1,
	sizeof(asn_DEF_CpmParameters_tags_1)
		/sizeof(asn_DEF_CpmParameters_tags_1[0]), /* 1 */
	asn_DEF_CpmParameters_tags_1,	/* Same as above */
	sizeof(asn_DEF_CpmParameters_tags_1)
		/sizeof(asn_DEF_CpmParameters_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_CpmParameters_1,
	4,	/* Elements count */
	&asn_SPC_CpmParameters_specs_1	/* Additional specs */
};

