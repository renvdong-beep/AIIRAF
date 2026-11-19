# -*- coding: utf-8 -*-
"""意图 Provider 选择顺序：真实模型优先，关键词仅作无模型回退。"""
from iraf_adapters.agentos.cli import build_intent_provider


def test_real_model_wins_over_keyword():
    p = build_intent_provider({
        "IRAF_AGENTOS_MODEL_URL": "http://127.0.0.1:9",
        "IRAF_AGENTOS_MODEL": "dummy",
        "IRAF_INTENT_KEYWORD": "1",
    })
    assert type(p).__name__ == "OpenAICompatibleIntentProvider"


def test_keyword_only_without_model():
    p = build_intent_provider({"IRAF_INTENT_KEYWORD": "1"})
    assert type(p).__name__ == "KeywordIntentProvider"


def test_no_provider_without_model_or_keyword():
    assert build_intent_provider({}) is None


def test_token_file_supported(tmp_path):
    key = tmp_path / "agentos.key"
    key.write_text("secret-token\n", encoding="utf-8")
    p = build_intent_provider({
        "IRAF_AGENTOS_MODEL_URL": "http://127.0.0.1:9",
        "IRAF_AGENTOS_MODEL": "dummy",
        "IRAF_AGENTOS_TOKEN_FILE": str(key),
    })
    assert p.config.token == "secret-token"
