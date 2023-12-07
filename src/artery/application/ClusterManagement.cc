#include "artery/application/ClusterManagement.h"
#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include <vanetza/asn1/vam.hpp>
#include "artery/application/Constants.hpp"
#include <omnetpp/simtime.h>
#include <string>
#include "artery/application/PersonDataProvider.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include "artery/traci/PersonController.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>
#include "traci/API.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "traci/Boundary.h"
#include "artery/traci/Cast.h"
#include "traci/Core.h"

namespace artery
{
namespace bg = boost::geometry;
using namespace omnetpp;

ClusterManagement::ClusterManagement(){}

void ClusterManagement::initialize(ClusterManagementParameters_t& cManagementPar){

	//Clustering options and parameters
	mNumCreateCluster = cManagementPar.NumCreateCluster;
	mMaxClusterDistance = cManagementPar.MaxClusterDistance;
	mMaxClusterVelocityDifference = cManagementPar.MaxClusterVelocityDifference;
	mMaxCombinedClusterDistance = cManagementPar.MaxCombinedClusterDistance;
	mMinClusterSize = cManagementPar.MinClusterSize;
	mMaxClusterSize = cManagementPar.MaxClusterSize;
	mNumClusterVAMRepeat = cManagementPar.NumClusterVAMRepeat;

	// Clustering timing parameters
	mTimeClusterUniquenessThreshold = cManagementPar.TimeClusterUniquenessThreshold;
	mTimeClusterBreakupWarning = cManagementPar.TimeClusterBreakupWarning;
	mTimeClusterJoinNotification = cManagementPar.TimeClusterJoinNotification;
	mTimeClusterJoinSuccess = cManagementPar.TimeClusterJoinSuccess;
	mTimeClusterIdChangeNotification = cManagementPar.TimeClusterIdChangeNotification;
	mTimeClusterIdPersist = cManagementPar.TimeClusterIdPersist;
	mTimeClusterContinuity = cManagementPar.TimeClusterContinuity;
	mTimeClusterLeaderWaiting = cManagementPar.TimeClusterLeaderWaiting;
	mTimeClusterLeaveNotification = cManagementPar.TimeClusterLeaveNotification;
	mTimeCombinedVruClusterOpportunity = cManagementPar.TimeCombinedVruClusterOpportunity; 
	
	mBoundingBoxShape = cManagementPar.DefaultBoundingBoxShape;
	mEnableBoundingBoxPadding = cManagementPar.EnableBoundingBoxPadding;

	mLastLeaderVamTimestamp = simTime();

	setCurrentVruRole(VruActiveStandalone);
	setBoundingBoxArea(0.0);
	setBoundingBoxPerimeter(0.0);
	setLargestDistanceToLeader(0.01);
}

void ClusterManagement::simObjects(const PersonDataProvider* pdp, artery::LocalDynamicMap* ldm, const traci::PersonController* pc){

	mPersonDataProvider =  pdp;
	mLocalDynamicMap = ldm;
	mPersonController = pc;

	api = mPersonController->getTraCI();
}

void ClusterManagement::setCurrentVruRole(VruRoleType newVruRole){currentVruRole = newVruRole;}

ClusterManagement::VruRoleType ClusterManagement::getCurrentVruRole(){return currentVruRole;}

bool ClusterManagement::isVruIdle(){return currentVruRole == VruIdle;}
bool ClusterManagement::isVruStandalone(){return currentVruRole == VruActiveStandalone;}
bool ClusterManagement::isVruClusterLeader(){return currentVruRole == VruActiveClusterLeader;}
bool ClusterManagement::isVruPassive(){return currentVruRole == VruPassive;}


bool ClusterManagement::vruShouldJoin(const vanetza::asn1::Vam& vam){
	ClusterParameters_t clusterPar = parseInformationContainer(vam);
	mDistFromCIC = clusterPar.distance;
	mCardFromCIC = clusterPar.cardinality;
	//return checkClusteringConditions(clusterPar.distance, clusterPar.velocityDifference, clusterPar.cardinality, clusterPar.withinBoundingBox);
	return (clusterPar.distance <= mMaxClusterDistance && clusterPar.velocityDifference <= mMaxClusterVelocityDifference && clusterPar.cardinality < mMaxClusterSize);
}

bool ClusterManagement::vruShouldLeave_VamParameters(const vanetza::asn1::Vam& vam){
	ClusterParameters_t clusterPar = parseInformationContainer(vam);

	if(clusterPar.cardinality > mMaxClusterSize){
		setLeaveReason(ClusterLeaveReason_notProvided);
		return true;

	//}else if(clusterPar.distance > mMaxClusterDistance || !clusterPar.withinBoundingBox){
	}else if(clusterPar.distance > mMaxClusterDistance){
		setLeaveReason(ClusterLeaveReason_outOfClusterBoundingBox);
		return true;

	}else if(clusterPar.velocityDifference < mMaxClusterVelocityDifference){
		setLeaveReason(ClusterLeaveReason_outOfClusterSpeedRange);
		return true;

	}else{
		return false;
	}

}

bool ClusterManagement::vruShouldLeave_ElapsedTime(const omnetpp::SimTime& T_now){
	if(T_now - mLastLeaderVamTimestamp >= timeClusterContinuity()){
		setLeaveReason(ClusterLeaveReason_clusterLeaderLost);
		return true;
	}else{
		return false;
	}
}

bool ClusterManagement::vruShouldCreateCluster(){
	// TODO: Should also evaluate if any available leader is nearby -> parse a set of "recent" VAMs 
	return (mInitialClusterMap.size() + 1) >= mNumCreateCluster;
}

bool ClusterManagement::clusterShouldBreakup_ElapsedTime(const omnetpp::SimTime& T_now){
	if(T_now - mClusterCreationTimestamp >= timeClusterLeaderWaiting()){
		setBreakupReason(ClusterBreakupReason_notProvided);
		return true;
	}else{
		return false;
	}
}

artery::ClusterManagement::pointXY ClusterManagement::getCurrentVruXY(){
	libsumo::TraCIPosition vruLeaderTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
	return pointXY(vruLeaderTraCIPos.x, vruLeaderTraCIPos.y);
}

artery::ClusterManagement::ClusterParameters_t ClusterManagement::parseInformationContainer(const vanetza::asn1::Vam& vam){

	const VruAwareness_t vam_data = (*vam).vam;
	ClusterParameters_t clusterParameters;
	clusterParameters.cardinality = vam_data.vamParameters.vruClusterInformationContainer->clusterCardinalitySize;

	double vruLeaderLon = vam_data.vamParameters.basicContainer.referencePosition.longitude/std::pow(10.0, 7);
	double vruLeaderLat = vam_data.vamParameters.basicContainer.referencePosition.latitude/std::pow(10.0, 7);

	libsumo::TraCIPosition leaderTraCIPos = api->simulation.convertGeo(vruLeaderLon,vruLeaderLat, true);
	pointXY leaderPos = pointXY(leaderTraCIPos.x,leaderTraCIPos.y);

	libsumo::TraCIPosition vruTraCIPos = position_cast(traci::Boundary(api->simulation.getNetBoundary()), mPersonDataProvider->position());
	pointXY vruPos = pointXY(vruTraCIPos.x, vruTraCIPos.y);

	SpeedValue_t vruLeaderSpeed = vam_data.vamParameters.vruHighFrequencyContainer->speed.speedValue;
	SpeedValue_t vruSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());

	clusterParameters.velocityDifference = velocityDifference(vruLeaderSpeed,vruSpeed);

	if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterPolygon){
		PolyPointList_t& receivedOffsetList = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList;
		clusterParameters.distance = bg::distance(vruPos, leaderPos)*vanetza::units::si::meter;
		Polygon_t receivedBoundingBox = reconstructPolygon(receivedOffsetList,leaderTraCIPos);
		clusterParameters.withinBoundingBox = bg::covered_by(vruPos, receivedBoundingBox);

	}else if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterCircle){
		std::tuple<long,long> centerOffset = getOffsetValues(*vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.nodeCenterPoint);
		pointXY centerXY = pointXY(bg::get<0>(leaderPos)+std::get<0>(centerOffset), bg::get<1>(leaderPos)+std::get<1>(centerOffset));
		clusterParameters.distance = bg::distance(vruPos, centerXY)*vanetza::units::si::meter;
		auto vamRadius = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.radius;
		clusterParameters.withinBoundingBox = (bg::distance(vruPos,centerXY) <= vamRadius/Radius_oneMeter) ? 1 : 0;		

	}else if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterRectangle){
		std::tuple<long,long> centerOffset = getOffsetValues(*vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterRectangle.nodeCenterPoint);
		pointXY centerXY = pointXY(bg::get<0>(leaderPos)+std::get<0>(centerOffset), bg::get<1>(leaderPos)+std::get<1>(centerOffset));
		long halfWidth = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterRectangle.semiMinorRangeLength;
		long halfLength = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeLength;
		long angle = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterRectangle.semiMajorRangeOrientation;
		Polygon_t receivedBoundingBox = reconstructRectangle(centerXY, halfWidth, halfLength, angle);
		clusterParameters.distance = bg::distance(vruPos, centerXY)*vanetza::units::si::meter;
		clusterParameters.withinBoundingBox = bg::covered_by(vruPos, receivedBoundingBox);
	}

	return clusterParameters;
	
}

std::tuple<long,long> ClusterManagement::getOffsetValues(const OffsetPoint_t& offsetPoint)
{
	long nodeX, nodeY;

	if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY1){
		Node_XY_20b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY1;
		nodeX = node.x;
		nodeY = node.y;
	}else if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY2){
		Node_XY_22b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY2;
		nodeX = node.x;
		nodeY = node.y;
	}else if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY3){
		Node_XY_24b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY3;
		nodeX = node.x;
		nodeY = node.y;
	}else if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY4){
		Node_XY_26b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY4;
		nodeX = node.x;
		nodeY = node.y;
	}else if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY5){
		Node_XY_28b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY5;
		nodeX = node.x;
		nodeY = node.y;
	}else if(offsetPoint.nodeOffsetPointxy.present == NodeOffsetPointXY_PR_node_XY6){
		Node_XY_32b_t node = offsetPoint.nodeOffsetPointxy.choice.node_XY6;
		nodeX = node.x;
		nodeY = node.y;
	}else{
		nodeX = 0;
		nodeY = 0;
	}

	return std::make_tuple(nodeX/100,nodeY/100);
} 

// Reconstructs Polygon Bounding Box based on offset list
artery::ClusterManagement::Polygon_t ClusterManagement::reconstructPolygon(const PolyPointList_t& receivedOffsetList, libsumo::TraCIPosition& leaderPos)
{
	std::vector<pointXY> reconstructedPositions;

	std::map<int,OffsetPoint_t*> offsetMap;
	for(int iter = 0; iter < receivedOffsetList.list.count; iter++){
		offsetMap.insert(std::pair<int, OffsetPoint_t*>(iter, receivedOffsetList.list.array[iter]));
	}

	bool firstIteration = true;
	pointXY previousPos;
	for(auto& offsetPoint : offsetMap){

		if(firstIteration){
			previousPos = pointXY(leaderPos.x, leaderPos.y);
			firstIteration = false;
		}

		std::tuple<long,long> nodeOffset = getOffsetValues(*offsetPoint.second); 
		pointXY currentPos = pointXY(bg::get<0>(previousPos) + std::get<0>(nodeOffset), bg::get<1>(previousPos) + std::get<1>(nodeOffset));
		reconstructedPositions.push_back(currentPos);
		previousPos = currentPos;
	}

	Polygon_t reconstructedPolygon;
	bg::assign_points(reconstructedPolygon, reconstructedPositions);
	return reconstructedPolygon;
}

artery::ClusterManagement::Polygon_t ClusterManagement::reconstructRectangle(const pointXY center, long halfWidth, long halfLength, long angle)
{
	Polygon_t reconstructedRectangle;

	double halfLengthMeters = halfLength/10.0;
	double halfWidthMeters = halfWidth/10.0;

	pointXY bottomLeftmost = pointXY(bg::get<0>(center)-halfLengthMeters, bg::get<1>(center)-halfWidthMeters);
	pointXY bottomRightmost = pointXY(bg::get<0>(center)+halfLengthMeters, bg::get<1>(center)-halfWidthMeters);
	pointXY topRightmost = pointXY(bg::get<0>(center)+halfLengthMeters, bg::get<1>(center)+halfWidthMeters);
	pointXY topLeftmost = pointXY(bg::get<0>(center)-halfLengthMeters, bg::get<1>(center)+halfWidthMeters);

	bg::append(reconstructedRectangle, bottomLeftmost);
	bg::append(reconstructedRectangle, bottomRightmost);
	bg::append(reconstructedRectangle, topRightmost);
	bg::append(reconstructedRectangle, topLeftmost);
	bg::append(reconstructedRectangle, bottomLeftmost);

	Polygon_t offsetPoly = translatePoly(reconstructedRectangle, -bg::get<0>(center), -bg::get<1>(center));
	Polygon_t rotatedRectangle = rotatePoly(offsetPoly, WGS84AngleToRadians(angle));
	reconstructedRectangle = translatePoly(rotatedRectangle, bg::get<0>(center), bg::get<1>(center));

	return reconstructedRectangle;
}

ClusterManagement::Polygon_t ClusterManagement::generatePolygonFromMap(VruClusterMap_t vruMap){
	Polygon_t resultingPolygon;
	pointXY vruLeaderPos = getCurrentVruXY(), furthestVru;
	double currentLargestDist = 0.0;

	bg::append(resultingPolygon, vruLeaderPos);
	for (auto const& itr : vruMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		pointXY vruPos = pointXY(vruTraCIPos.x, vruTraCIPos.y);
		bg::append(resultingPolygon, vruPos);
		double comparableDist = bg::comparable_distance(vruLeaderPos,vruPos);
		if(comparableDist > currentLargestDist){
			currentLargestDist = comparableDist;
			furthestVru = vruPos;
		}
	}

	setLargestDistanceToLeader(bg::distance(furthestVru, vruLeaderPos));

	bg::append(resultingPolygon, vruLeaderPos);

	bg::remove_spikes(resultingPolygon);
	bg::unique(resultingPolygon);
	bg::correct(resultingPolygon);

	return resultingPolygon;
}

std::tuple<long,long> ClusterManagement::calculateOffset(pointXY pointA, pointXY pointB){
	// NodeOffsets are given in centimeters
	long offsetX = (bg::get<0>(pointB) - bg::get<0>(pointA))*100.0;
	long offsetY = (bg::get<1>(pointB) - bg::get<1>(pointA))*100.0;

	return std::make_tuple(offsetX, offsetY);
}

std::vector<std::tuple<long,long>> ClusterManagement::calculatePolyOffsetList(Polygon_t basePoly)
{
	std::vector<pointXY> clusterPositions;
	pointXY previousPos;
	std::vector<std::tuple<long,long>> distancesList;

	// Using boost geometry, simplifies the original polygon with convex hull algorithm
	Polygon_t hull;
	bg::convex_hull(basePoly, hull);

	if(mEnableBoundingBoxPadding){
		hull = applyPaddingToPoly(hull, calculatePaddingAmount(mClusterMap));
	}

	bg::remove_spikes(hull);
	bg::unique(hull);
	bg::correct(hull);

	clusterPositions = hull.outer();

	setBoundingBoxArea(bg::area(hull));
	setBoundingBoxPerimeter(bg::perimeter(hull));

	// Obtains list of offsets
	previousPos = getCurrentVruXY();
	for (auto const& currentPos : clusterPositions){
		distancesList.push_back(calculateOffset(previousPos,currentPos));
		previousPos = currentPos;
	}

	return distancesList;
}

// TODO: Change to calculate both speed and heading
double ClusterManagement::velocityDifference(SpeedValue_t referenceSpeed, SpeedValue_t evaluatedSpeed)
{
	return (referenceSpeed > 0) ? std::abs(static_cast<double>(referenceSpeed - evaluatedSpeed))/referenceSpeed : 0.0;
}

bool ClusterManagement::checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox)
{
	//return (cardinality < mMaxClusterSize && velocityDifference <= mMaxClusterVelocityDifference && (distance <= mMaxClusterDistance || withinBoundingBox));
	return (cardinality < mMaxClusterSize && velocityDifference <= mMaxClusterVelocityDifference && distance <= mMaxClusterDistance);
}

void ClusterManagement::createVruClusterMap(){
	
	mInitialClusterMap.clear();

	std::vector<double> mNearbyVruDistances;

	auto lDM = mLocalDynamicMap->localMap();
			
	//calculate distance between egoVRU and received VRU
	double VruEgoLon = ((Constants::round(mPersonDataProvider->longitude(), Constants::microdegree) * Longitude_oneMicrodegreeEast) / std::pow( 10.0, 7 ));
	double VruEgoLat = ((Constants::round(mPersonDataProvider->latitude(), Constants::microdegree) * Latitude_oneMicrodegreeNorth) / std::pow( 10.0, 7 )); 
	libsumo::TraCIPosition vruLeaderTraCIPos = api->simulation.convertGeo(VruEgoLon,VruEgoLat, true);
	SpeedValue_t VruEgoSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());

	//collect data from the VRUs in the LDM and update mVruClusterMap
	for (auto const& itr : *lDM) {
									
		double mVlatitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));				
		double mVlongitude = ((itr.second.object.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));			
		libsumo::TraCIPosition mVTraCIPosition = api->simulation.convertGeo(mVlongitude, mVlatitude, true);
		vanetza::units::Length distanceVRU = bg::distance(pointXY(vruLeaderTraCIPos.x,vruLeaderTraCIPos.y), pointXY(mVTraCIPosition.x,mVTraCIPosition.y)) * vanetza::units::si::meter;	

		SpeedValue_t mVSpeed = itr.second.object.asn1()->vam.vamParameters.vruHighFrequencyContainer->speed.speedValue;
		double clusterVelocityDifference = velocityDifference(VruEgoSpeed, mVSpeed);

		//update mVruClusterMap when distance, velocity and cardinality constraints are met		
		if(checkClusteringConditions(distanceVRU,clusterVelocityDifference,mInitialClusterMap.size()+1) && !itr.second.object.asn1()->vam.vamParameters.vruClusterInformationContainer) {	

			mNearbyVruDistances.push_back(distanceVRU/vanetza::units::si::meter);

			uint32_t stationID = itr.second.object.asn1()->header.stationID;
			artery::VaObject awr = itr.second.object;
			mInitialClusterMap.insert(std::pair<LocalDynamicMap::StationID, artery::VaObject>(stationID, awr));
		}
	}

	if(mNearbyVruDistances.size() > 0){
		double smallestDist = *std::min_element(mNearbyVruDistances.begin(), mNearbyVruDistances.end());
		mSmallestDistanceToVru = (smallestDist > 0.01 ? smallestDist : 0.01);
	}

}

void ClusterManagement::updateVruClusterMap(const vanetza::asn1::Vam& vam, std::string operation){

	auto vruLon = (*vam).vam.vamParameters.basicContainer.referencePosition.longitude/std::pow(10.0,7);
	auto vruLat = (*vam).vam.vamParameters.basicContainer.referencePosition.latitude/std::pow(10.0,7);

	libsumo::TraCIPosition vruTraCI = api->simulation.convertGeo(vruLon, vruLat, true);
	pointXY vruXY = pointXY(vruTraCI.x, vruTraCI.y);

	pointXY leaderXY = getCurrentVruXY();

	if(operation == "include_VRU" && bg::distance(vruXY, leaderXY) <= 5.0){
		if(mClusterMap.size()+1 < mMaxClusterSize){
			mClusterMap.insert(std::pair<LocalDynamicMap::StationID, artery::VaObject>(vam->header.stationID, VaObject(vam)));
		}
	}else if(operation == "remove_VRU" && !mClusterMap.empty()){
		mClusterMap.erase(vam->header.stationID);
	}
}

ClusterManagement::pointXY ClusterManagement::calculateCenterFromBasePoly(Polygon_t basePoly){
	
	Box_t envelope;
	pointXY centerPoint;

	bg::envelope(basePoly, envelope);
	bg::centroid(envelope, centerPoint);
	
	return centerPoint;
}

double ClusterManagement::calculateRadius(VruClusterMap_t vruMap, pointXY center){

	double currentLargestDist = 0.0;
	pointXY furthestVru;

	for (auto const& itr : vruMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		pointXY vruPos = pointXY(vruTraCIPos.x, vruTraCIPos.y);
		double comparableDist = bg::comparable_distance(vruPos, center);
		if(comparableDist >= currentLargestDist){
			currentLargestDist = comparableDist;
			furthestVru = vruPos;
		}
	}

	auto radiusMeters = bg::distance(furthestVru, center)*vanetza::units::si::meter;

	if(mEnableBoundingBoxPadding){
		radiusMeters += calculatePaddingAmount(mClusterMap)*vanetza::units::si::meter;
	}

	double radiusDecimeters = Constants::round(radiusMeters, Constants::decimeter);

	return radiusDecimeters <= 0.0 ? 1.0 : radiusDecimeters;
}

double ClusterManagement::radiansToWGS84Angle(double radians){
	double WGSAngle = radians * 1800.0 / M_PI;
	WGSAngle = 1800.0 - WGSAngle; 
	if(WGSAngle < 0){
		WGSAngle += 3600.0;
	} 
	return WGSAngle;
}

double ClusterManagement::WGS84AngleToRadians(double WGSAngle){
	return WGSAngle * M_PI / 1800.0;
}

ClusterManagement::Polygon_t ClusterManagement::translatePoly(const Polygon_t originalPoly, double offsetX, double offsetY){
	Polygon_t translatedPoly;
	bg::strategy::transform::translate_transformer<double, 2, 2> applyOffset(offsetX, offsetY);
	bg::transform(originalPoly, translatedPoly, applyOffset);
	return translatedPoly;
}

ClusterManagement::Polygon_t ClusterManagement::rotatePoly(const Polygon_t originalPoly, double angle){
	Polygon_t rotatedPoly;
	bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> applyRotation(angle);
	bg::transform(originalPoly, rotatedPoly, applyRotation);
	return rotatedPoly;
}

std::tuple<ClusterManagement::Polygon_t, double>  ClusterManagement::getRotatedRectangle(Polygon_t basePoly){

	Box_t outputEnvelope;
	double minArea = std::numeric_limits<double>::max(), optimalAngle;

	pointXY basePolyCenter = calculateCenterFromBasePoly(basePoly);
	Polygon_t offsetPoly = translatePoly(basePoly, -bg::get<0>(basePolyCenter), -bg::get<1>(basePolyCenter));

	for (double currentAngle = 0.0; currentAngle <= 2*M_PI; currentAngle += 0.1) {

		Box_t candidateEnvelope;
		Polygon_t rotatedPoly = rotatePoly(offsetPoly, currentAngle);
		bg::envelope(rotatedPoly, candidateEnvelope);
        double rotatedArea = bg::area(candidateEnvelope);

        if (rotatedArea < minArea) {
            minArea = rotatedArea;
			outputEnvelope = candidateEnvelope;
			optimalAngle = currentAngle;
        }
    }

	Polygon_t baseRectangle;
	bg::convert(outputEnvelope, baseRectangle);

	if(mEnableBoundingBoxPadding){
		baseRectangle = applyPaddingToPoly(baseRectangle, calculatePaddingAmount(mClusterMap));
	}

	Polygon_t outputRectangle = rotatePoly(baseRectangle, -optimalAngle);
	Polygon_t finalRectangle = translatePoly(outputRectangle, bg::get<0>(basePolyCenter), bg::get<1>(basePolyCenter));

	return std::make_tuple(finalRectangle, radiansToWGS84Angle(optimalAngle));
}

std::string ClusterManagement::selectBoundingBoxShape(Polygon_t basePoly){

	//TODO: Develop a less reduntant way to obtain areas -> current solution repeats plenty of calculations
	Polygon_t hullPoly;
	bg::convex_hull(basePoly, hullPoly);
	double areaPoly = bg::area(hullPoly);
	
	pointXY circleCenter = calculateCenterFromBasePoly(basePoly);
	double circleRadius = calculateRadius(mClusterMap, circleCenter)/Radius_oneMeter;
	double areaCircle = M_PI*std::pow(circleRadius,2);

	// Cluster shape selection based on highest Num_Clusters/m^2 (i.e. smaller area)
	std::string selectedShape = areaPoly <= areaCircle ? "Polygon" : "Circular";

	return selectedShape;
}

InformationContainerData_t ClusterManagement::generateDataForNewCluster(long id){
	InformationContainerData_t infoContainerData;
	infoContainerData.shape = "Circular";
	infoContainerData.cardinalitySize = mMinClusterSize;
	double smallestValidDistance;
	if(getSmallestDistanceToVru()*.5 < 0.1){
		smallestValidDistance = 0.1*Radius_oneMeter;
	}else{
		smallestValidDistance = getSmallestDistanceToVru()*Radius_oneMeter*.5;
	}
	infoContainerData.circleRadius = smallestValidDistance; 
	infoContainerData.clusterId = id;
	registerClusterId(infoContainerData.clusterId);

	setBoundingBoxArea(M_PI*std::pow(infoContainerData.circleRadius/Radius_oneMeter,2));
	setBoundingBoxPerimeter(2*M_PI*infoContainerData.circleRadius/Radius_oneMeter);
	setLargestDistanceToLeader(0.0);

	return infoContainerData;
}

InformationContainerData_t ClusterManagement::generateDataForCurrentCluster(){
	InformationContainerData_t infoContainerData;
	infoContainerData.cardinalitySize = getCurrentClusterSize();
	infoContainerData.clusterId = getClusterId();

	Polygon_t basePoly = generatePolygonFromMap(mClusterMap);

	boost::geometry::validity_failure_type shapeFailureType;
	bg::is_valid(basePoly, shapeFailureType);

	if(shapeFailureType == bg::failure_few_points){
		infoContainerData.nodeOffset = std::make_tuple(0,0);
		infoContainerData.circleRadius = 1.0;
		setBoundingBoxArea(M_PI*std::pow(infoContainerData.circleRadius/Radius_oneMeter,2));
		setBoundingBoxPerimeter(2*M_PI*infoContainerData.circleRadius/Radius_oneMeter);

	}else if(shapeFailureType == bg::no_failure || shapeFailureType == bg::failure_self_intersections){

		if(mBoundingBoxShape == "Autoselect"){
			mBoundingBoxShape = selectBoundingBoxShape(basePoly);
		}

		if(mBoundingBoxShape == "Polygon"){
			infoContainerData.polyOffsets = calculatePolyOffsetList(basePoly);
		}else if(mBoundingBoxShape == "Circular"){
			pointXY centerPoint = calculateCenterFromBasePoly(basePoly);
			infoContainerData.nodeOffset = calculateOffset(getCurrentVruXY(), centerPoint);
			infoContainerData.circleRadius = calculateRadius(mClusterMap, centerPoint);
			setBoundingBoxArea(M_PI*std::pow(infoContainerData.circleRadius/Radius_oneMeter,2));
			setBoundingBoxPerimeter(2*M_PI*infoContainerData.circleRadius/Radius_oneMeter);
		}else if(mBoundingBoxShape == "Rectangular"){
			Polygon_t rectangle;
			double angle;
			std::tie(rectangle, angle) = getRotatedRectangle(basePoly);
			infoContainerData.nodeOffset = calculateOffset(getCurrentVruXY(), calculateCenterFromBasePoly(rectangle));
			infoContainerData.rectangleLength = bg::distance(rectangle.outer()[0], rectangle.outer()[1])*SemiRangeLength_oneMeter*.5;
			infoContainerData.rectangleWidth = bg::distance(rectangle.outer()[1], rectangle.outer()[2])*SemiRangeLength_oneMeter*.5;
			infoContainerData.orientationAngle = angle;
			setBoundingBoxArea(bg::area(rectangle));
			setBoundingBoxPerimeter(bg::perimeter(rectangle));
		}
	}

	infoContainerData.shape = mBoundingBoxShape;
	
	return infoContainerData;
}

double ClusterManagement::calculatePaddingAmount(const VruClusterMap_t clusterMap){

	double maxSpeed = 0.0;

	for(auto& vru : clusterMap){
		double vruSpeed = vru.second.asn1()->vam.vamParameters.vruHighFrequencyContainer->speed.speedValue;
		maxSpeed = std::max(vruSpeed, maxSpeed);
	}

	// 150ms determined experimentally
	auto messageTime = 150.0*std::pow(10.0,-3);

	return (maxSpeed/100.0)*messageTime;
}

ClusterManagement::Polygon_t ClusterManagement::applyPaddingToPoly(const Polygon_t originalPoly, double paddingAmount){

	// Prevents null padding and shrinkage of the shape
	if(paddingAmount <= 0.0){
		return originalPoly;
	}
	
	bg::strategy::buffer::distance_symmetric<double> distance_strategy(paddingAmount);
    bg::strategy::buffer::join_miter join_strategy;
	bg::strategy::buffer::end_flat end_strategy;
    bg::strategy::buffer::point_square square_strategy;
    bg::strategy::buffer::side_straight side_strategy;
	MultiPoly_t paddedShape;

	bg::buffer(originalPoly, paddedShape, distance_strategy, side_strategy, join_strategy, end_strategy, square_strategy);
	bg::correct(paddedShape[0]);
	bg::unique(paddedShape[0]);

	// Prevents padded rectangles from becoming polygons with > 4 vertices
	if(mBoundingBoxShape == "Rectangular" && bg::num_segments(paddedShape[0]) > 4){
		Polygon_t simplifiedShape;
		bg::simplify(paddedShape[0], simplifiedShape, paddingAmount);

		return simplifiedShape.outer().size() > 0 ? simplifiedShape : paddedShape[0];
	}

	return paddedShape[0];

}

void ClusterManagement::updateReceivedIdMap(long receivedId, const omnetpp::SimTime& T_now){
	
	auto searchItr = mReceivedIdMap.find(receivedId);
	if(searchItr != mReceivedIdMap.end() && !mReceivedIdMap.empty()){
		searchItr->second = T_now;
	}else{
		mReceivedIdMap.insert(std::make_pair(receivedId, T_now));
	}
}

bool ClusterManagement::isClusterIdUnique(long proposedId){
	return (mReceivedIdMap.find(proposedId) == mReceivedIdMap.end() || mReceivedIdMap.empty());
}

void ClusterManagement::refreshReceivedIdMap(const omnetpp::SimTime& T_now){

	if(!mReceivedIdMap.empty()){
		for(auto refreshItr = mReceivedIdMap.begin(); refreshItr != mReceivedIdMap.end();){
			if(T_now - refreshItr->second > mTimeClusterLeaderWaiting){
				refreshItr = mReceivedIdMap.erase(refreshItr);
			}else{
				++refreshItr;
			}
		}
	}
}

}