"""Unit tests for expressions.py - safe math expression evaluator."""

import pytest
import math

from orion_sdk.parser.expressions import safe_eval, safe_eval_or_none, safe_eval_or_default


class TestSafeEval:
    """Tests for safe_eval function."""

    def test_integer(self):
        assert safe_eval("42") == 42.0

    def test_float(self):
        assert safe_eval("3.14159") == pytest.approx(3.14159)

    def test_negative(self):
        assert safe_eval("-5") == -5.0

    def test_addition(self):
        assert safe_eval("2 + 3") == 5.0

    def test_subtraction(self):
        assert safe_eval("10 - 4") == 6.0

    def test_multiplication(self):
        assert safe_eval("6 * 7") == 42.0

    def test_division(self):
        assert safe_eval("22 / 7") == pytest.approx(3.142857, rel=1e-5)

    def test_power_caret(self):
        """Test ^ for exponentiation (ProtoGen style)."""
        assert safe_eval("2^8") == 256.0
        assert safe_eval("2^15") == 32768.0
        assert safe_eval("2^16") == 65536.0

    def test_power_double_star(self):
        """Test ** for exponentiation (Python style)."""
        assert safe_eval("2**8") == 256.0
        assert safe_eval("2**15") == 32768.0

    def test_parentheses(self):
        assert safe_eval("(2 + 3) * 4") == 20.0
        assert safe_eval("2 * (3 + 4)") == 14.0

    def test_pi_constant(self):
        assert safe_eval("pi") == pytest.approx(math.pi)

    def test_e_constant(self):
        assert safe_eval("e") == pytest.approx(math.e)

    def test_pi_in_expression(self):
        """Test pi in expressions - common in ProtoGen XML."""
        assert safe_eval("pi/2") == pytest.approx(math.pi / 2)
        assert safe_eval("2*pi") == pytest.approx(2 * math.pi)
        assert safe_eval("-3*pi/2") == pytest.approx(-3 * math.pi / 2)

    def test_complex_scaler_expression(self):
        """Test actual ProtoGen scaler expression: 180*10000000/pi."""
        result = safe_eval("180*10000000/pi")
        expected = 180 * 10000000 / math.pi
        assert result == pytest.approx(expected)

    def test_nested_parentheses(self):
        assert safe_eval("((2 + 3) * (4 + 5))") == 45.0

    def test_unary_minus(self):
        assert safe_eval("-pi") == pytest.approx(-math.pi)
        assert safe_eval("--5") == 5.0

    def test_unary_plus(self):
        assert safe_eval("+5") == 5.0

    def test_empty_string(self):
        assert safe_eval("") == 0.0
        assert safe_eval("   ") == 0.0

    def test_whitespace(self):
        assert safe_eval("  42  ") == 42.0
        assert safe_eval(" 2 + 3 ") == 5.0

    def test_floor_division(self):
        assert safe_eval("7 // 2") == 3.0

    def test_modulo(self):
        assert safe_eval("10 % 3") == 1.0

    def test_invalid_expression(self):
        with pytest.raises(ValueError):
            safe_eval("2 +")

    def test_unknown_variable(self):
        with pytest.raises(ValueError):
            safe_eval("x + 1")

    def test_function_call_rejected(self):
        """Ensure function calls are rejected for security."""
        with pytest.raises(ValueError):
            safe_eval("sin(pi)")

    def test_import_rejected(self):
        """Ensure import statements are rejected."""
        with pytest.raises(ValueError):
            safe_eval("__import__('os')")


class TestSafeEvalOrNone:
    """Tests for safe_eval_or_none function."""

    def test_valid_expression(self):
        assert safe_eval_or_none("42") == 42.0

    def test_none_input(self):
        assert safe_eval_or_none(None) is None

    def test_empty_string(self):
        assert safe_eval_or_none("") is None
        assert safe_eval_or_none("   ") is None


class TestSafeEvalOrDefault:
    """Tests for safe_eval_or_default function."""

    def test_valid_expression(self):
        assert safe_eval_or_default("42") == 42.0

    def test_none_input(self):
        assert safe_eval_or_default(None) == 0.0
        assert safe_eval_or_default(None, 99.0) == 99.0

    def test_empty_string(self):
        assert safe_eval_or_default("") == 0.0
        assert safe_eval_or_default("", 123.0) == 123.0


class TestProtoGenExpressions:
    """Test actual expressions found in ProtoGen XML files."""

    def test_scaler_lat_lon(self):
        """scaler="180*10000000/pi" for lat/lon encoding."""
        scaler = safe_eval("180*10000000/pi")
        # Should be approximately 572957795.13
        assert scaler == pytest.approx(572957795.13, rel=1e-6)

    def test_max_pi(self):
        """max="pi" for angle fields."""
        assert safe_eval("pi") == pytest.approx(math.pi)

    def test_min_negative_pi(self):
        """min="-3*pi/2" for tilt angle."""
        result = safe_eval("-3*pi/2")
        assert result == pytest.approx(-3 * math.pi / 2)

    def test_power_of_two(self):
        """max="2^15" for 16-bit signed max."""
        assert safe_eval("2^15") == 32768.0
        assert safe_eval("2^15-1") == 32767.0

    def test_scaler_100(self):
        """scaler="100" simple integer scaler."""
        assert safe_eval("100") == 100.0

    def test_scaler_1000(self):
        """scaler="1000.0" float scaler."""
        assert safe_eval("1000.0") == 1000.0

    def test_scaler_10000(self):
        """scaler="10000" for altitude encoding."""
        assert safe_eval("10000") == 10000.0
