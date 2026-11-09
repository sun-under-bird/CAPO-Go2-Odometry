from pathlib import Path
import subprocess
import tempfile
import unittest


class TimestampPredictionTest(unittest.TestCase):
    def test_sampling_rate_and_instance_isolation(self):
        repo = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            executable = str(Path(tmp) / "prediction_probe")
            sources = ["tests/timestamp_prediction_probe.c"] + [
                "FusionEstimator/Estimators/"+name for name in
                ("StateSpaceModel_Go2.c", "Estimator1001_Kalman.c", "matrix.c")]
            subprocess.run(["gcc", "-std=c99", "-O2", "-I", str(repo),
                            *[str(repo / x) for x in sources], "-lm", "-o", executable], check=True)
            subprocess.run([executable], check=True)


if __name__ == "__main__":
    unittest.main()
