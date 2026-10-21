import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import xmlschema
from lxml import etree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_schema_comply import XmlValidation

SCHEMA_DIR = Path(__file__).resolve().parents[1] / "resources" / "schema"
SOURCE = SCHEMA_DIR / "OpenSCENARIOv1.4.xsd"
OUTPUT = SCHEMA_DIR / "OpenSCENARIOv1.4-esmini.xsd"


class EsminiSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.standard = xmlschema.XMLSchema(SOURCE)
        cls.extended = xmlschema.XMLSchema11(OUTPUT)
        cls.lxml_standard = etree.XMLSchema(etree.parse(str(SOURCE)))

    def test_wrapper_only_overrides_parameter_and_expression(self):
        root = etree.parse(str(OUTPUT)).getroot()
        ns = {"xsd": "http://www.w3.org/2001/XMLSchema"}
        self.assertEqual(
            root.get("{http://www.w3.org/2007/XMLSchema-versioning}minVersion"), "1.1"
        )
        self.assertEqual(
            [etree.QName(child).localname for child in root],
            ["annotation", "override"],
        )
        override = root.find("xsd:override", ns)
        self.assertEqual(override.get("schemaLocation"), SOURCE.name)
        self.assertEqual(
            [(etree.QName(child).localname, child.get("name")) for child in override],
            [("simpleType", "parameter"), ("simpleType", "expression")],
        )

    def test_numeric_attribute_boundary(self):
        cases = [
            ("2.4", True, True),
            ("$Width", True, True),
            ("${$Width / 2}", True, True),
            ("$CarWhite::Width", False, True),
            ("${$CarWhite::Width / 2 + 0.5}", False, True),
            ("${$CarWhite::Width + $CarBlue::Width}", False, True),
            ("$CarWhite:Width", False, False),
            ("${$CarWhite:Width / 2}", False, False),
            ("${$CarWhite:::Width / 2}", False, False),
            ("$CarWhite::", False, False),
            ("$CarWhite::Width::Extra", False, False),
            ("$1Car::Width", False, False),
        ]
        for value, standard, extended in cases:
            with self.subTest(value=value):
                document = etree.Element("OpenSCENARIO")
                etree.SubElement(
                    document,
                    "FileHeader",
                    revMajor="1",
                    revMinor="4",
                    date="2026-10-21T00:00:00",
                    description="Schema test",
                    author="esmini",
                )
                etree.SubElement(document, "ParameterDeclarations")
                etree.SubElement(document, "CatalogLocations")
                etree.SubElement(document, "RoadNetwork")
                etree.SubElement(document, "Entities")
                storyboard = etree.SubElement(document, "Storyboard")
                etree.SubElement(etree.SubElement(storyboard, "Init"), "Actions")
                stop = etree.SubElement(storyboard, "StopTrigger")
                group = etree.SubElement(stop, "ConditionGroup")
                condition = etree.SubElement(
                    group, "Condition", name="Stop", delay="0", conditionEdge="none"
                )
                by_value = etree.SubElement(condition, "ByValueCondition")
                etree.SubElement(
                    by_value, "SimulationTimeCondition", value=value, rule="greaterThan"
                )
                self.assertEqual(self.standard.is_valid(document), standard)
                self.assertEqual(self.extended.is_valid(document), extended)
                self.assertEqual(self.lxml_standard.validate(document), standard)

    def test_schema_selection_is_explicit_and_version_specific(self):
        standard = XmlValidation()
        extended = XmlValidation(esmini_extensions=True)
        self.assertEqual(
            Path(standard.get_xsd_to_validate("4", "test.xosc")).resolve(), SOURCE
        )
        self.assertEqual(
            Path(extended.get_xsd_to_validate("4", "test.xosc")).resolve(), OUTPUT
        )
        for revision, filename in (("3", "test.xosc"), ("4", "test.xodr")):
            self.assertEqual(
                standard.get_xsd_to_validate(revision, filename),
                extended.get_xsd_to_validate(revision, filename),
            )

    def test_cli_requires_explicit_opt_in(self):
        document = """<OpenSCENARIO xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:noNamespaceSchemaLocation="OpenSCENARIOv1.4-esmini.xsd">
            <FileHeader revMajor="1" revMinor="4" date="2026-10-21T00:00:00"
                description="Schema test" author="esmini"/>
            <CatalogLocations/><RoadNetwork/><Entities/>
            <Storyboard><Init><Actions/></Init><StopTrigger><ConditionGroup>
                <Condition name="Stop" delay="0" conditionEdge="none"><ByValueCondition>
                    <SimulationTimeCondition value="${$Car1::Width / 2}" rule="greaterThan"/>
                </ByValueCondition></Condition>
            </ConditionGroup></StopTrigger></Storyboard>
        </OpenSCENARIO>"""
        script = SOURCE.parents[2] / "scripts" / "run_schema_comply.py"
        with tempfile.TemporaryDirectory() as directory:
            scenario = Path(directory) / "qualified.xosc"
            scenario.write_text(document, encoding="utf-8")
            for options, expected in (([], 1), (["--esmini-extensions"], 0)):
                with self.subTest(options=options):
                    result = subprocess.run(
                        [sys.executable, str(script), *options, str(scenario)],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    self.assertEqual(
                        result.returncode, expected, result.stdout + result.stderr
                    )


if __name__ == "__main__":
    unittest.main()
