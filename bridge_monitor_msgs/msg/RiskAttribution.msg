string dimension
float64 raw_score
float64 weighted_score
float64 weight
bool is_primary_driver

# M3 provenance: invalid sources carry zero effective weight and are never
# interpreted as a healthy zero score.
bool source_valid
string validity
string reason_code
string provenance
