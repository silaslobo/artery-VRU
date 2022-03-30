package traci;

//
// This policy enables only vehicles located within defined regions of interest.
// This is useful when running large scenarios.
//
simple RegionOfInterestPersonPolicy like PersonPolicy
{
    parameters:
        @class(traci::RegionOfInterestPersonPolicy);
        xml regionsOfInterest = default(xml("<regions />"));
}
