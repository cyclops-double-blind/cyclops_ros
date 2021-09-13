int32 id
float32 u
float32 v

# A 2x2 information weight. Column-major order is preferred, but the ordering is
# not much relevant since information matrices are symmetric.
float32[4] weight
