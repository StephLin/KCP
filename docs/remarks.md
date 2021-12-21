# Remarks

## Tuning Parameters

The major parameters are

- `kcp::KCP::Params::k` and
- [`kcp::KCP::Params::teaser::noise_bound`](https://teaser.readthedocs.io/en/latest/api-cpp.html#_CPPv4N6teaser24RobustRegistrationSolver6Params11noise_boundE),

where `k` is the number of nearest points of each source point selected to be
part of initial correspondences, and `noise_bound` is the criterion to determine
if a correspondence is correct. In our paper, we suggest `k=2` and `noise_bound`
the 3-sigma (we use `noise_bound=0.06` meters for nuScenes data), and those are
default values in the library.

To use different parameters to the KCP solver, please refer to the following
snippets:

<b>C++</b>

```cpp
#include <kcp/solver.hpp>

auto params = kcp::KCP::Params();

params.k                  = 2;
params.teaser.noise_bound = 0.06;

auto solver = kcp::KCP(params);
```

**Python**

```python
import pykcp

params = pykcp.KCPParams()
params.k = 2
params.teaser.noise_bound = 0.06

solver = pykcp.KCP(params)
```

## Controlling Computational Cost

Instead of
[correspondence-free registration in TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus/issues/120),
KCP considers k closest point correspondences to reduce the major computational
cost of the maximum clique algorithm, and we have expressed the ability for
real-world scenarios without any complicate or learning-based feature descriptor
in the paper. However, it is still possible to encounter computational time or
memory issue if there are too many correspondences fed to the solver.

We suggest controlling your keypoints around 500 for k=2 (in this way the
computational time will be much closer to the one presented in the paper).

## Torwarding Global Registration Approaches

It is promising that KCP can be extended to a global registration approach if a
fast and reliable sparse feature point representation method is employed.

In this way, the role of RANSAC, a fast registration approach usually used in
learning based approaches, is similar to KCP's, but the computation results of
KCP are deterministic, and also, KCP has better theoretical supports.
