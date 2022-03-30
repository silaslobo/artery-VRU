package artery.envmod;

import artery.envmod.LocalEnvironmentModel;
import artery.inet.Person;

module Person extends artery.inet.Person
{
    submodules:
        environmentModel: LocalEnvironmentModel {
            @display("p=214,57");
            middlewareModule = default(".middleware");
        }
}
