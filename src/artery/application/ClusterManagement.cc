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

	setCurrentVruRole(VruActiveStandalone);
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
	return checkClusteringConditions(clusterPar.distance, clusterPar.velocityDifference, clusterPar.cardinality, clusterPar.withinBoundingBox);
}

bool ClusterManagement::vruShouldLeave(const vanetza::asn1::Vam& vam){
	ClusterParameters_t clusterPar = parseInformationContainer(vam);
	return !checkClusteringConditions(clusterPar.distance, clusterPar.velocityDifference, clusterPar.cardinality, clusterPar.withinBoundingBox);
}

bool ClusterManagement::vruShouldCreateCluster(){
	// TODO: Should also evaluate if any available leader is nearby -> parse a set of "recent" VAMs 
	return (mInitialClusterMap.size() + 1) >= mNumCreateCluster;
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

	clusterParameters.distance = bg::distance(vruPos, leaderPos)*vanetza::units::si::meter;

	SpeedValue_t vruLeaderSpeed = vam_data.vamParameters.vruHighFrequencyContainer->speed.speedValue;
	SpeedValue_t vruSpeed = buildVAMSpeedValue(mPersonDataProvider->speed());

	clusterParameters.velocityDifference = velocityDifference(vruLeaderSpeed,vruSpeed);

	if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterPolygon){
		PolyPointList_t& receivedOffsetList = vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterPolygon.polyPointList;

		Polygon_t receivedBoundingBox = reconstructPolygon(receivedOffsetList,leaderTraCIPos);
		clusterParameters.withinBoundingBox = bg::within(vruPos, receivedBoundingBox);
		// if(clusterParameters.withinBoundingBox == 1){
		// 	std::cout << "Reconstructed (cluster " << vam_data.vamParameters.vruClusterInformationContainer->clusterId << "): " << bg::dsv(receivedBoundingBox) << std::endl;
		// }

	}else if(vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.present == ClusterBoundingBoxShape_PR_clusterCircle){
		std::tuple<long,long> centerOffset = getOffsetValues(*vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.nodeCenterPoint);
		pointXY centerXY = pointXY(bg::get<0>(leaderPos)+std::get<0>(centerOffset), bg::get<1>(leaderPos)+std::get<1>(centerOffset));
		clusterParameters.withinBoundingBox = (bg::distance(vruPos,centerXY) <= vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.radius) ? 1 : 0;
		// if(clusterParameters.withinBoundingBox == 1 && vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.radius > 0){
		// 	std::cout << "Reconstructed cluster " << vam_data.vamParameters.vruClusterInformationContainer->clusterId << std::endl;
		// 	std::cout << "Parsed radius: " << vam_data.vamParameters.vruClusterInformationContainer->clusterBoundingBoxShape.choice.clusterCircle.radius << std::endl;
		// 	std::cout << "Parsed center: (" << bg::get<0>(centerXY) << "," << bg::get<1>(centerXY) << ") \n" << std::endl;
		// }
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

ClusterManagement::Polygon_t ClusterManagement::generatePolygonFromMap(VruClusterMap_t vruMap){
	Polygon_t resultingPolygon;
	pointXY vruLeaderPos = getCurrentVruXY();

	//std::cout << "ID: " << getClusterId() << std::endl;
	//std::cout << "Members: (" << mPersonDataProvider->getStationId() << "*";
	bg::append(resultingPolygon, vruLeaderPos);
	for (auto const& itr : vruMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		bg::append(resultingPolygon, pointXY(vruTraCIPos.x, vruTraCIPos.y));
		//std::cout << ", " << itr.first;
	}
	//std::cout << ")" << std::endl;
	bg::append(resultingPolygon, vruLeaderPos);

	bg::correct(resultingPolygon);
	bg::unique(resultingPolygon);
	// std::cout << "Poly from map: " << std::setprecision(15) << bg::dsv(resultingPolygon) << std::endl;
	// std::cout << "Area: " << bg::area(resultingPolygon)*std::pow(10.0,-4) << " cm2" << std::endl;
	// std::cout << "Cardinality: " << vruMap.size()+1 << " VRUs" << std::endl;
	// std::cout << "Density: " << bg::area(resultingPolygon)/(vruMap.size()+1) << " VRUs/cm2" << std::endl;
	return resultingPolygon;
}

std::tuple<long,long> ClusterManagement::calculateOffset(pointXY pointA, pointXY pointB){
	// NodeOffsets are given in centimeters
	long offsetX = (bg::get<0>(pointB) - bg::get<0>(pointA))*100.0;
	long offsetY = (bg::get<1>(pointB) - bg::get<1>(pointA))*100.0;

	//std::cout << "(" << offsetX << "," << offsetY << "), " ;

	return std::make_tuple(offsetX, offsetY);
}

std::vector<std::tuple<long,long>> ClusterManagement::calculatePolyOffsetList(VruClusterMap_t vruMap)
{
	std::vector<pointXY> clusterPositions;
	pointXY previousPos;
	std::vector<std::tuple<long,long>> distancesList;

	Polygon_t clusterPolygon = generatePolygonFromMap(vruMap);

	// Using boost geometry, simplifies the original polygon with convex hull algorithm
	Polygon_t hull;
	bg::convex_hull(clusterPolygon, hull);
	//std::cout << "Hull poly: " << bg::dsv(hull) << std::endl;
	clusterPositions = hull.outer();

	// Obtains list of offsets
	//std::cout << "Offset: "; 
	previousPos = getCurrentVruXY();
	for (auto const& currentPos : clusterPositions){
		distancesList.push_back(calculateOffset(previousPos,currentPos));
		previousPos = currentPos;
	}
	//std::cout << "\n" << std::endl;

	return distancesList;
}

// TODO: Change to calculate both speed and heading
double ClusterManagement::velocityDifference(SpeedValue_t referenceSpeed, SpeedValue_t evaluatedSpeed)
{
	return (referenceSpeed > 0) ? std::abs(static_cast<double>(referenceSpeed - evaluatedSpeed))/referenceSpeed : 0.0;
}

bool ClusterManagement::checkClusteringConditions(vanetza::units::Length distance, double velocityDifference, long cardinality, bool withinBoundingBox)
{
	return (cardinality < mMaxClusterSize && velocityDifference <= mMaxClusterVelocityDifference && (distance <= mMaxClusterDistance || withinBoundingBox));
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
		mSmallestDistanceToVru = *std::min_element(mNearbyVruDistances.begin(), mNearbyVruDistances.end());
	}

}

void ClusterManagement::updateVruClusterMap(const vanetza::asn1::Vam& vam, std::string operation){

	if(operation == "include_VRU"){
		if(mClusterMap.size()+1 < mMaxClusterSize){
			mClusterMap.insert(std::pair<LocalDynamicMap::StationID, artery::VaObject>(vam->header.stationID, VaObject(vam)));
		}
	}else if(operation == "remove_VRU" && !mClusterMap.empty()){
		mClusterMap.erase(vam->header.stationID);
	}
}

ClusterManagement::pointXY ClusterManagement::calculateCenterFromMap(VruClusterMap_t vruMap){
	
	Polygon_t pointsFromMap = generatePolygonFromMap(vruMap);
	bg::model::box<pointXY> envelope;
	pointXY centerPoint;

	bg::envelope(pointsFromMap, envelope);
	bg::centroid(envelope, centerPoint);

	//std::cout << "Leader: (" << bg::get<0>(getCurrentVruXY()) << "," << bg::get<1>(getCurrentVruXY()) << ")" << std::endl;
	//std::cout << "Centroid: (" << bg::get<0>(centerPoint) << "," << bg::get<1>(centerPoint) << ")" << std::endl;
	
	return centerPoint;
}

double ClusterManagement::calculateRadius(VruClusterMap_t vruMap, pointXY center){
	std::vector<double> distanceList;

	for (auto const& itr : vruMap) {
		auto vruLon = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.longitude) / std::pow( 10.0, 7 ));
		auto vruLat = ((itr.second.asn1()->vam.vamParameters.basicContainer.referencePosition.latitude) / std::pow( 10.0, 7 ));
		libsumo::TraCIPosition vruTraCIPos = api->simulation.convertGeo(vruLon,vruLat, true);
		distanceList.push_back(bg::distance(center, pointXY(vruTraCIPos.x, vruTraCIPos.y)));		
	}
	
	double radius = ceil(*std::max_element(distanceList.begin(), distanceList.end())*Radius_oneMeter);
	//std::cout << "Radius: " << radius << " dm \n" << std::endl;
	return radius;
}

InformationContainerData_t ClusterManagement::generateDataForNewCluster(long id){
	InformationContainerData_t infoContainerData;
	infoContainerData.shape = "Circular";
	infoContainerData.cardinalitySize = mMinClusterSize;
	infoContainerData.circleRadius = getSmallestDistanceToVru()*Radius_oneMeter*.5; 
	infoContainerData.clusterId = id;
	registerClusterId(infoContainerData.clusterId);
	return infoContainerData;
}

InformationContainerData_t ClusterManagement::generateDataForCurrentCluster(){
	InformationContainerData_t infoContainerData;
	infoContainerData.cardinalitySize = getCurrentClusterSize();
	infoContainerData.clusterId = getClusterId();
	infoContainerData.shape = mBoundingBoxShape;

	if(mBoundingBoxShape == "Polygon"){
		infoContainerData.polyOffsets = calculatePolyOffsetList(getVruClusterMap());
	}else if(mBoundingBoxShape == "Circular"){
		pointXY centerPoint = calculateCenterFromMap(mClusterMap);
		infoContainerData.nodeOffset = calculateOffset(getCurrentVruXY(), centerPoint);
		infoContainerData.circleRadius = calculateRadius(mClusterMap, centerPoint);
	}
	
	return infoContainerData;
}

}