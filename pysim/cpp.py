# Low-level C++ fidelity helpers: float32 arithmetic, the truncating epsilon
# comparators (Utility.cpp:206-225), the per-link LCG, CCSVParser and the
# DTASettings INI reader (mfc_compat GetPrivateProfileString semantics).
import re

import numpy as np

f32 = np.float32


def less_eq(a, b):
    # bool less_eq(double a, double b): (long)(a * 100) < (long)(b * 100) + 2
    return int(float(a) * 100.0) < int(float(b) * 100.0) + 2


def less_than(a, b):
    # bool less_than(double a, double b): (long)(a * 100) < (long)(b * 100)
    return int(float(a) * 100.0) < int(float(b) * 100.0)


def floating_less(a, b):
    # bool g_floating_point_value_less_than(float a, float b), Utility.cpp:240:
    # (long)(a * g_precision_constant2) < (long)(b * g_precision_constant2),
    # g_precision_constant2 = 100. Arguments are float, product is float.
    return int(f32(a) * f32(100)) < int(f32(b) * f32(100))


class LinkRNG:
    # DTALink::g_rtnRandomM (DTALite.h), LCG a=17364, c=0, M=65521,
    # seeded with m_LinkNo (DTALite.cpp, link reading: m_RandomSeed = m_LinkNo).
    __slots__ = ("seed",)

    def __init__(self, seed):
        self.seed = int(seed)

    def next_percent(self):
        # g_rtnRandomM: returns int in [0, 100]
        if self.seed > 65521:
            self.seed %= 65521
        self.seed = (self.seed * 17364) % 65521
        return int(self.seed * 1.0 / 65521 * 100 + 0.5)

    def ratio(self):
        # DTALink::GetRandomRatio (DTALite.h:1568): float(seed) / 65521
        self.seed = (self.seed * 17364) % 65521
        return f32(self.seed) / f32(65521)


_int_prefix = re.compile(r"\s*([+-]?\d+)")
_float_prefix = re.compile(
    r"\s*([+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?)")


def parse_int(text):
    # istringstream >> int: longest valid integer prefix; None on failure
    m = _int_prefix.match(text)
    if m is None:
        return None
    return int(m.group(1))


def parse_float(text):
    # istringstream >> float/double: longest valid numeric prefix
    m = _float_prefix.match(text)
    if m is None:
        return None
    return float(m.group(1))


class ProgramStop(Exception):
    # g_ProgramStop(): the C++ code aborts the run on malformed inputs
    pass


_MASK32 = 0xFFFFFFFF


class WellRng:
    # WELLRNG512a (Utility.cpp:1082-1127), seeded by InitWELLRNG512a with
    # state[k] = k + g_RandomSeed at the top of g_ReadInputFiles.
    def __init__(self, seed):
        self.state = [(k + seed) & _MASK32 for k in range(16)]
        self.state_i = 0

    def ratio(self):
        # double WELLRNG512a(): returns STATE[i] * 2.32830643653869628906e-10
        s = self.state
        i = self.state_i
        z0 = s[(i + 15) & 0xF]
        v0 = s[i]
        vm1 = s[(i + 13) & 0xF]
        vm2 = s[(i + 9) & 0xF]
        z1 = ((v0 ^ ((v0 << 16) & _MASK32)) ^
              (vm1 ^ ((vm1 << 15) & _MASK32))) & _MASK32
        z2 = (vm2 ^ (vm2 >> 11)) & _MASK32
        new_v1 = (z1 ^ z2) & _MASK32
        new_v0 = ((z0 ^ ((z0 << 2) & _MASK32)) ^
                  (z1 ^ ((z1 << 18) & _MASK32)) ^
                  ((z2 << 28) & _MASK32) ^
                  (new_v1 ^ (((new_v1 << 5) & _MASK32) & 0xDA442D24))) & _MASK32
        s[i] = new_v1
        s[(i + 15) & 0xF] = new_v0
        self.state_i = (i + 15) & 0xF
        return s[self.state_i] * 2.32830643653869628906e-10


class VehicleGenerationRng:
    # g_GetRandomRatioForVehicleGeneration (Utility.cpp:277): LCG on the
    # global g_RandomSeedForVehicleGeneration = 101, float result.
    def __init__(self):
        self.seed = 101

    def ratio(self):
        self.seed = (17364 * self.seed) % 65521
        return f32(self.seed) / f32(65521)


def parse_line(line):
    # CCSVParser::ParseLine (CSVParser.cpp:125): strip one trailing CR, then
    # plain comma split (getline semantics: no trailing empty field) when the
    # line has no double quote, else the quote-aware walk of the C++ code.
    if line.endswith("\r"):
        line = line[:-1]
    if '"' not in line:
        if line == "":
            return []
        fields = line.split(",")
        if line.endswith(","):
            fields = fields[:-1]
        return fields
    fields = []
    while len(line) > 0:
        n1 = line.find(",")
        n2 = line.find('"')
        if n1 == -1 and n2 == -1:
            fields.append(line)
            break
        if n1 == -1 and n2 != -1:
            n3 = line.find('"', n2 + 1)
            fields.append(line[n2 + 1:n3])
            break
        if n1 != -1 and (n2 == -1 or n1 < n2):
            fields.append(line[:n1])
            if n1 < len(line) - 1:
                line = line[n1 + 1:]
            else:
                fields.append("")
                break
        else:  # quote before comma
            n3 = line.find('"', n2 + 1)
            fields.append(line[n2 + 1:n3])
            idx = line.find(",", n3 + 1)
            if idx == -1:
                break
            line = line[idx + 1:]
    return fields


class CsvParser:
    # CCSVParser (CSVParser.h/.cpp): first line is the header; field names
    # have leading spaces stripped, later duplicate names win.
    def __init__(self, path):
        self.path = str(path)
        # latin-1 keeps a 1:1 byte mapping like the C++ char stream
        with open(self.path, "r", newline="", encoding="latin-1") as fh:
            raw_lines = fh.read().split("\n")
        # std::getline drops the newline; a final trailing newline leaves a
        # last "" element that getline would never return as a record.
        if raw_lines and raw_lines[-1] == "":
            raw_lines = raw_lines[:-1]
        self._lines = raw_lines
        self.field_index = {}
        if self._lines:
            for i, name in enumerate(parse_line(self._lines[0])):
                self.field_index[name.lstrip(" ")] = i
        self._pos = 1
        self.fields = []

    def read_record(self):
        # ReadRecord(): an empty raw line ends reading (returns False)
        self.fields = []
        if self._pos >= len(self._lines):
            return False
        line = self._lines[self._pos]
        self._pos += 1
        if len(line) == 0:
            return False
        self.fields = parse_line(line)
        return True

    def get(self, name, kind, required=False):
        # GetValueByFieldName / ...Required. Returns (ok, value):
        # ok False leaves the caller's value untouched in C++; here value None.
        if name not in self.field_index:
            if required:
                raise ProgramStop(
                    "field %s missing in %s" % (name, self.path))
            return False, None
        idx = self.field_index[name]
        if len(self.fields) == 0 or idx >= len(self.fields):
            return False, None
        text = self.fields[idx]
        if len(text) == 0:
            return False, None
        if kind == "str":
            return True, text
        if kind == "int":
            value = parse_int(text)
        elif kind == "float":
            value = parse_float(text)
            if value is not None:
                value = f32(value)
        elif kind == "double":
            value = parse_float(text)
        else:
            raise ValueError(kind)
        if value is None:
            return False, None
        return True, value


class IniFile:
    # DTASettings.txt via the Linux GetPrivateProfileString shim
    # (mfc_compat.h:238): case-sensitive section/key, trimmed, first hit wins.
    def __init__(self, path):
        self.values = {}
        try:
            with open(path, "r", newline="") as fh:
                lines = fh.read().split("\n")
        except OSError:
            lines = []
        section = ""
        for raw in lines:
            text = raw.strip()
            if not text or text[0] in ";#":
                continue
            if text.startswith("[") and text.endswith("]"):
                section = text[1:-1].strip()
                continue
            eq = text.find("=")
            if eq == -1:
                continue
            key = text[:eq].strip()
            if (section, key) not in self.values:
                self.values[(section, key)] = text[eq + 1:].strip()

    def get_int(self, section, key, default):
        text = self.values.get((section, key))
        if text is None:
            return default
        value = parse_int(text)
        return default if value is None else value

    def get_float(self, section, key, default):
        text = self.values.get((section, key))
        if text is None:
            return f32(default)
        value = parse_float(text)
        return f32(default) if value is None else f32(value)

    def get_str(self, section, key, default):
        return self.values.get((section, key), default)
