Challenger vs AlphaZero Training – README
=========================================

Overview
-----------

This module trains a **Challenger AI agent** to defeat a **frozen AlphaZero model** in Quoridor.

Unlike standard self-play, the Challenger learns exclusively by playing against a strong, fixed opponent. This avoids self-play collapse and ensures all learning signals come from meaningful, high-quality gameplay.

Training Strategy
--------------------

### Key Idea

*   AlphaZero is **pretrained and frozen**
    
*   Challenger starts from scratch
    
*   All training data comes from **Challenger vs AlphaZero games**
    

### Training Loop

1.  Run parallel games: Challenger (MCTS) vs AlphaZero (MCTS)
    
2.  Collect **only Challenger moves**
    
3.  Store in replay buffer
    
4.  Train Challenger network via SGD
    
5.  Periodically evaluate vs AlphaZero (no noise)
    
6.  Save best-performing model
    

Key Features
---------------

*   Frozen AlphaZero opponent (no updates)
    
*   Increased exploration (Dirichlet noise) for Challenger
    
*   CNN + Reinforcement Learning (DQN-style policy/value learning)
    
*   Replay buffer for stable training
    
*   Soft target updates (stability)
    
*   Alternating sides (fair training)
    
*   Evaluation tournaments for real performance tracking
    

🚀 How to Run
-------------

From project root:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   python -m quoridor_alphazero.train_vs_alphazero \    --az-model-dir ./quoridor_az_models/run1_19800_batch_1024 \    --challenger-dir ./quoridor_challenger_models/run1 \    --episodes 20000   `

📊 Understanding Training Logs
------------------------------

Example:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   ep=108  az_sims=64  ply=18.6  buf=944    W/L/D=0/106/2  wr(100)=0.0%    loss=1.953 (p=1.917 v=0.003)  dt=75s   `

### Fields Explained

*   **ep**: Total games played
    
*   **az\_sims**: AlphaZero MCTS simulations
    
*   **ply**: Average game length (half-moves)
    
*   **buf**: Replay buffer size
    

### Performance Metrics

*   **W/L/D**: Wins / Losses / Draws (Challenger)
    
*   **wr(100)**: Win rate over last 100 decisive games
    

### Loss Metrics

*   **loss**: Total loss
    
*   **p**: Policy loss (move prediction error)
    
*   **v**: Value loss (win/loss prediction error)
    

### Timing

*   **dt**: Time elapsed
    

TensorBoard Metrics
----------------------

Launch:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   tensorboard --logdir=./quoridor_challenger_tensorboard/   `

### Key Graphs

#### Win Rate (challenger/win\_rate\_100)

*   Starts at 0%
    
*   Should increase over time
    
*   Main indicator of learning
    

#### Policy Loss (loss/policy)

*   Should decrease early
    
*   Indicates learning of good moves
    

#### Value Loss (loss/value)

*   Starts near 0 (agent always loses)
    
*   Should increase slightly as agent improves
    

#### Game Length (game/avg\_length\_100)

*   Early: short games (agent loses fast)
    
*   Later: longer games (agent resists)
    

#### Buffer Size (buffer/size)

*   Should steadily increase
    

#### Evaluation Win Rate (eval/win\_rate)

*   True performance (no randomness)
    
*   Increases in steps, not smoothly
    

Expected Training Behaviour
------------------------------

### Phase 1 (0–500 episodes)

*   0% win rate 
    
*   Policy loss decreasing 
    
*   Value loss near 0 
    
*   Short games
    

### Phase 2 (500–1500 episodes)

*   First occasional wins
    
*   Value loss increases slightly
    
*   Game length increases
    

### Phase 3 (1500+ episodes)

*   Win rate begins rising
    
*   Agent discovers effective strategies
    
*   Faster improvement
    

When to Worry
----------------

*   Policy loss stops decreasing early → learning stalled
    
*   Games always very short (<10 ply) → instant-loss behaviour
    
*   Win rate still 0 after ~3000 episodes → opponent too strong
    
*   Loss becomes NaN → training instability
    

Key Insights
---------------

*   Early **0% win rate is normal**
    
*   Low value loss ≠ intelligence (just predicting consistent losses)
    
*   Policy loss is the **main learning signal early on**
    
*   Improvements often happen in **sudden jumps**, not gradually
    

Summary
----------

This training setup:

*   Produces **stronger, more robust agents**
    
*   Avoids **self-play collapse**
    
*   Forces the model to learn **against a fixed high-level opponent**
    

At early stages, the Challenger is not failing — it is **bootstrapping from zero**.

Outputs
----------

*   Challenger weights → --challenger-dir
    
*   Best model → /best directory
    
*   TensorBoard logs → --tb-log-dir
    

Final System
---------------

The trained model integrates into:

*   Perception → Board state extraction
    
*   CNN → Feature extraction
    
*   RL → Decision making
    
*   Robot → Physical move execution
    

Final Note
-------------

Learning against a strong fixed opponent is harder than self-play, but leads to:

*   More stable training
    
*   Stronger policies
    
*   Better real-world performance