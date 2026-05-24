# Quoridor AI Benchmark

Round-robin tournament suite for the three Quoridor AI engines in this project:

| Agent | File | Requires |
|---|---|---|
| **Engine** | `minimax_policy.py` | nothing — pure Python |
| **DQN** | `dqn_agent.py` + `dqn_model.py` | `qnet.weights.h5` checkpoint |
| **AlphaZero** | `az_network.py` + `mcts.py` | `az_net.weights.h5` checkpoint |

---

## Install

```bash
pip install -e .           # installs quoridor-benchmark (no TF)
pip install -e ".[neural]" # also installs TensorFlow for DQN / AZ
```

---

## Quickstart (no weights needed)

```bash
python example_no_weights.py
```

Or via the CLI:

```bash
python -m quoridor_benchmark \
    --engine depth=3 \
    --engine depth=1 \
    --games 30 \
    --output ./results
```

---

## Full three-way comparison

```bash
python -m quoridor_benchmark \
    --engine "depth=3" \
    --dqn "/path/to/quoridor_models/latest" \
    --alphazero "/path/to/quoridor_az_models/latest,sims=64" \
    --games 50 \
    --output ./results
```

---

## Python API

```python
from quoridor_benchmark import run_benchmark, BenchmarkConfig, generate_report
from quoridor_benchmark.agents import (
    load_engine_agent,
    load_dqn_agent,
    load_alphazero_agent,
)

agents = {
    "Engine": load_engine_agent(depth=3),
    "DQN":    load_dqn_agent("/path/to/quoridor_models/latest"),
    "AZ":     load_alphazero_agent("/path/to/quoridor_az_models/latest", n_simulations=64),
}

result = run_benchmark(agents, config=BenchmarkConfig(games_per_pair=50))
generate_report(result, output_dir="./results")
```

---

## Output

The report shows:

- **Elo rankings** (iterative update, K=32, base=1500)
- **Win / Loss / Draw counts** and win rates
- **Illegal move count** (model quality signal)
- **Move timing** (avg + max ms per agent)
- **Head-to-head win rate matrix**
- **Shortest / longest game** info

Files written to `--output`:

| File | Contents |
|---|---|
| `report.txt` | Full text report |
| `results.json` | Structured JSON with all stats |
| `h2h.csv` | Head-to-head win rates (one row per ordered pair) |

---

## Package structure

```
quoridor_benchmark/
├── __init__.py          # public API
├── __main__.py          # CLI entry point
├── benchmark.py         # round-robin runner + Elo
├── match.py             # single-game runner
├── agents.py            # load_engine / load_dqn / load_alphazero
├── report.py            # console + file report
│
│   ── copied from project source ──
├── quoridor_utils.py    # board, rules, move types
├── encoder.py           # state tensor + action encoding
├── minimax_policy.py    # alpha-beta engine
├── dqn_agent.py         # DQN agent (load/save/select_action)
├── dqn_model.py         # Q-network architecture
├── az_network.py        # AlphaZero dual-head ResNet
└── mcts.py              # PUCT MCTS
```
