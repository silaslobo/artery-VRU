/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "../cpm.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "IntersectionTopologyPosition.h"

static asn_TYPE_member_t asn_MBR_IntersectionTopologyPosition_1[] = {
	{ ATF_POINTER, 2, offsetof(struct IntersectionTopologyPosition, laneID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaneID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"laneID"
		},
	{ ATF_POINTER, 1, offsetof(struct IntersectionTopologyPosition, laneOffset),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_OffsetPoint,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"laneOffset"
		},
};
static const int asn_MAP_IntersectionTopologyPosition_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_IntersectionTopologyPosition_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IntersectionTopologyPosition_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* laneID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* laneOffset */
};
static asn_SEQUENCE_specifics_t asn_SPC_IntersectionTopologyPosition_specs_1 = {
	sizeof(struct IntersectionTopologyPosition),
	offsetof(struct IntersectionTopologyPosition, _asn_ctx),
	asn_MAP_IntersectionTopologyPosition_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_IntersectionTopologyPosition_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	2,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IntersectionTopologyPosition = {
	"IntersectionTopologyPosition",
	"IntersectionTopologyPosition",
	&asn_OP_SEQUENCE,
	asn_DEF_IntersectionTopologyPosition_tags_1,
	sizeof(asn_DEF_IntersectionTopologyPosition_tags_1)
		/sizeof(asn_DEF_IntersectionTopologyPosition_tags_1[0]), /* 1 */
	asn_DEF_IntersectionTopologyPosition_tags_1,	/* Same as above */
	sizeof(asn_DEF_IntersectionTopologyPosition_tags_1)
		/sizeof(asn_DEF_IntersectionTopologyPosition_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_IntersectionTopologyPosition_1,
	2,	/* Elements count */
	&asn_SPC_IntersectionTopologyPosition_specs_1	/* Additional specs */
};

