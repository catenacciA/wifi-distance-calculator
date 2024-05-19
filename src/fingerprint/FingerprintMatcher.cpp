#include "../../include/fingerprint/FingerprintMatcher.h"

FingerprintMatcher::FingerprintMatcher(const FingerprintDatabase &db,
                                       ISimilarityCalculator &calc)
    : database(db), calculator(calc) {}

std::string FingerprintMatcher::match(
    const std::vector<std::pair<std::string, int>> &currentRSSIValues) const {
  const auto &db = database.getDatabase();
  std::string bestMatch;
  double bestSimilarity = std::numeric_limits<double>::max();

  for (const auto &[cellId, fingerprint] : db) {
    double similarity = calculator.calculate(fingerprint, currentRSSIValues);
    if (similarity < bestSimilarity) {
      bestSimilarity = similarity;
      bestMatch = cellId;
    }
  }

  return bestMatch;
}
