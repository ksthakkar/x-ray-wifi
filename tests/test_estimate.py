import unittest

from hub.serve.estimate import Estimate, EstimateValidationError


def valid_payload() -> dict:
    return {
        "t_us": 1_738_450_123_456_789,
        "site": "lab-partition-a",
        "present": 0.91,
        "position_m": [1.42, 1.85],
        "covariance": [[0.18, 0.03], [0.03, 0.44]],
        "height_m": None,
        "nodes_online": [1, 2, 4, 5],
        "nodes_expected": [1, 2, 3, 4, 5],
        "quality": 0.78,
        "model": "fake@1",
    }


class EstimateTests(unittest.TestCase):
    def test_accepts_architecture_contract(self) -> None:
        estimate = Estimate.from_mapping(valid_payload())
        self.assertEqual(estimate.position_m, (1.42, 1.85))
        self.assertEqual(estimate.nodes_online, (1, 2, 4, 5))

    def test_rejects_missing_fields(self) -> None:
        payload = valid_payload()
        del payload["quality"]
        with self.assertRaisesRegex(EstimateValidationError, "missing fields: quality"):
            Estimate.from_mapping(payload)

    def test_rejects_invalid_covariance(self) -> None:
        payload = valid_payload()
        payload["covariance"] = [[0.1, 0.4], [0.4, 0.1]]
        with self.assertRaisesRegex(EstimateValidationError, "positive semidefinite"):
            Estimate.from_mapping(payload)

    def test_rejects_duplicate_nodes(self) -> None:
        payload = valid_payload()
        payload["nodes_online"] = [1, 1]
        with self.assertRaisesRegex(EstimateValidationError, "duplicates"):
            Estimate.from_mapping(payload)

    def test_rejects_non_finite_values(self) -> None:
        payload = valid_payload()
        payload["position_m"] = [float("nan"), 1.0]
        with self.assertRaisesRegex(EstimateValidationError, "finite"):
            Estimate.from_mapping(payload)


if __name__ == "__main__":
    unittest.main()
