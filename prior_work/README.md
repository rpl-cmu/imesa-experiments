# Prior Work
This module contains implementations of prior works used for comparison against iMESA. Below are descriptions of each supported prior work.

## Structure
* `baselines/` - Contains implementation of the Centralized and Independent baselines
* `ddfsam2/` - Contains implementation of DDF-SAM2

#### Centralized
This baseline represents that which could be achieved if all robots communicate their measurements back to a central server. Under the communication model assumed in the paper above, such an architecture is impossible. However, a centralized solver provides a level of accuracy that we strive to match with distributed methods. Thus it is an excellent baseline for comparison. Note: to implement the centralized solver within the interface of `IncrementalSAMAgent` we had to get fancy with `static` variables. Modify the this implementation with care.

#### Independent
This baseline represents that which could be achieved if robots never communicate, and instead each independently optimize their local factor-graph. This baseline should represent an upper-bound on the performance of a distributed method.

#### DDF-SAM2
This prior work was originally presented in the paper "DDF-SAM 2.0: Consistent distributed smoothing and mapping" by Cunningham et al. in 2012. In this approach robots share potentials being careful to avoid double counting of information. A significant downside to DDF-SAM2 compared to iMESA is that DDF-SAM2 does not enforce robots converge to a single solution, and the necessity to compute marginals negatively affects its runtime efficiency. Note: we use the naive bayes approximation as suggested by Cunningham et al.