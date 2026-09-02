import unittest
from iraf_core.registry import SkillManifest,SkillRegistry

class SemVerTest(unittest.TestCase):
    def setUp(self):
        self.registry=SkillRegistry()
        for version in ("1.9.0","1.10.0","2.0.0"):
            self.registry.register(SkillManifest("demo",version,version,"development",(),(),(),1,"unsupported",(),"safe",{}, {}, ()))

    def test_numeric_order_and_constraints(self):
        self.assertEqual("1.10.0",self.registry.resolve("demo","<2.0.0").manifest.version)
        self.assertEqual("1.10.0",self.registry.resolve("demo","^1.0.0").manifest.version)
        self.assertEqual("1.9.0",self.registry.resolve("demo","~1.9.0").manifest.version)
        self.assertEqual("1.9.0",self.registry.resolve("demo","1.9.0").manifest.version)
        self.assertEqual("2.0.0",self.registry.resolve("demo").manifest.version)

if __name__=="__main__":
    unittest.main()
