//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple VaService like ItsG5Service
{
    parameters:
        @class(VaService);
        @signal[VamReceived](type=artery::VaObject);
        @signal[VamSent](type=artery::VaObject);

        @statistic[reception](source=VamReceived;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);
        @statistic[transmission](source=VamSent;record=vector(vamStationId)?,vector(vamGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering VAM generation (in degree)
        double headingDelta = default(0.2);

        // change in position triggering VAM generation
        double positionDelta @unit(m) = default(0.5m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);
        
        bool ignorePersons = false;

}
