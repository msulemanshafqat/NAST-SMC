"""
Figure 6: Interactive Comparative RMSE Performance Dashboard
IEEE Transactions Journal Format
Neural-AS-NST-STSMC vs Baselines across Exp 1-4

Requirements:
  pip install plotly pandas numpy kaleido

Export to PDF/PNG:
  fig.write_image("fig6_comparative_rmse.pdf", width=1200, height=700)
  fig.write_html("fig6_comparative_rmse.html")  # Interactive HTML
"""

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd
import numpy as np

# ============================================================================
# DATA: From Tables I-V (Experiments 1-4)
# ============================================================================

# Experiment 1: Nominal (mean of 4 trajectories: spiral, circle, square, random)
exp1_nominal = {
    'Neural-AS-NST-STSMC': {'rmse_pos': 4.48, 'rmse_att': 0.044, 'energy': 5.97, 'iae': 0.054},
    'STSMC': {'rmse_pos': 8.07, 'rmse_att': 0.035, 'energy': 5.97, 'iae': 0.108},
    'ISMC': {'rmse_pos': 8.04, 'rmse_att': 0.053, 'energy': 25.59, 'iae': 0.107},
    'FracFITSMC': {'rmse_pos': 8.16, 'rmse_att': 0.059, 'energy': 5.96, 'iae': 0.110},
    'DISMC': {'rmse_pos': 8.04, 'rmse_att': 0.046, 'energy': 5.97, 'iae': 0.109},
    'PD': {'rmse_pos': 14.86, 'rmse_att': 0.426, 'energy': 10.53, 'iae': 0.210}
}

# Experiment 2: 60% Fault
exp2_fault60 = {
    'Neural-AS-NST-STSMC': {'rmse_pos': 5.41, 'rmse_att': 0.015, 'energy': 7.00, 'iae': 0.086},
    'STSMC': {'rmse_pos': 9.62, 'rmse_att': 0.003, 'energy': 7.02, 'iae': 0.152},
    'ISMC': {'rmse_pos': 9.53, 'rmse_att': 0.006, 'energy': 25.59, 'iae': 0.151},
    'FracFITSMC': {'rmse_pos': 9.67, 'rmse_att': 0.010, 'energy': 7.01, 'iae': 0.153},
    'DISMC': {'rmse_pos': 9.64, 'rmse_att': 0.005, 'energy': 7.02, 'iae': 0.152},
    'PD': {'rmse_pos': 17.26, 'rmse_att': 0.379, 'energy': 11.73, 'iae': 0.268}
}

# Experiment 3: Wind Disturbance
exp3_wind = {
    'Neural-AS-NST-STSMC': {'rmse_pos': 7.05, 'rmse_att': 0.019, 'energy': 6.00, 'iae': 0.121},
    'STSMC': {'rmse_pos': 12.32, 'rmse_att': 0.004, 'energy': 6.14, 'iae': 0.210},
    'ISMC': {'rmse_pos': 12.27, 'rmse_att': 0.011, 'energy': 25.57, 'iae': 0.209},
    'FracFITSMC': {'rmse_pos': 12.45, 'rmse_att': 0.017, 'energy': 6.13, 'iae': 0.214},
    'DISMC': {'rmse_pos': 12.35, 'rmse_att': 0.014, 'energy': 6.14, 'iae': 0.212},
    'PD': {'rmse_pos': 18.42, 'rmse_att': 0.381, 'energy': 10.42, 'iae': 0.322}
}

# Experiment 4: 5 ms Delay (stable only)
exp4_delay5ms = {
    'Neural-AS-NST-STSMC': {'rmse_pos': 4.56, 'rmse_att': 0.018, 'energy': 5.98, 'iae': 0.073},
    'FracFITSMC': {'rmse_pos': 8.11, 'rmse_att': 0.015, 'energy': 6.18, 'iae': 0.128},
    'STSMC': {'rmse_pos': 8.24, 'rmse_att': 0.083, 'energy': 25.27, 'iae': 0.165},
    'ISMC': {'rmse_pos': 8.05, 'rmse_att': 0.038, 'energy': 24.98, 'iae': 0.127},
    'DISMC': {'rmse_pos': 8.11, 'rmse_att': 0.054, 'energy': 24.94, 'iae': 0.128},
    'PD': {'rmse_pos': 194.07, 'rmse_att': 0.035, 'energy': 25.54, 'iae': 0.308}
}

# ============================================================================
# CREATE MULTI-PANEL SUBPLOT FIGURE (4 experiments)
# ============================================================================

fig = make_subplots(
    rows=2, cols=2,
    subplot_titles=(
        '<b>Exp. 1: Nominal Tracking</b><br><sub>Mean of 4 trajectories</sub>',
        '<b>Exp. 2: Severe Fault</b><br><sub>60% thrust loss (3 rotors)</sub>',
        '<b>Exp. 3: Wind Disturbance</b><br><sub>8-12 m/s gusts + noise</sub>',
        '<b>Exp. 4: Actuator Delay</b><br><sub>5 ms ESC latency</sub>'
    ),
    specs=[[{'secondary_y': False}, {'secondary_y': False}],
           [{'secondary_y': False}, {'secondary_y': False}]],
    horizontal_spacing=0.12,
    vertical_spacing=0.15
)

# Define controller colors (consistent with draft)
colors = {
    'Neural-AS-NST-STSMC': '#1f77b4',  # Blue (primary)
    'STSMC': '#ff7f0e',                  # Red/Orange (SOTA)
    'ISMC': '#2ca02c',                   # Green
    'FracFITSMC': '#d62728',             # Red
    'DISMC': '#9467bd',                  # Purple
    'PD': '#7f7f7f'                      # Gray (baseline)
}

controller_order = ['Neural-AS-NST-STSMC', 'STSMC', 'ISMC', 'FracFITSMC', 'DISMC', 'PD']

# ────────────────────────────────────────────────────────────────────────
# SUBPLOT 1: Exp 1 - Nominal
# ────────────────────────────────────────────────────────────────────────
for i, ctrl in enumerate(controller_order):
    rmse = exp1_nominal[ctrl]['rmse_pos']
    fig.add_trace(
        go.Bar(
            x=[ctrl],
            y=[rmse],
            marker=dict(color=colors[ctrl], line=dict(width=1.5, color='white')),
            hovertemplate=f'<b>{ctrl}</b><br>RMSE: {rmse:.2f} mm<br>Energy: {exp1_nominal[ctrl]["energy"]:.2f} Wh<extra></extra>',
            name=ctrl,
            # showlegend=(i == 0),  # Show legend only for first subplot (this shows for single trace)
            showlegend=True,  # Show legend only for first subplot
            legendgroup=ctrl
        ),
        row=1, col=1
    )

# ────────────────────────────────────────────────────────────────────────
# SUBPLOT 2: Exp 2 - Fault
# ────────────────────────────────────────────────────────────────────────
for i, ctrl in enumerate(controller_order):
    rmse = exp2_fault60[ctrl]['rmse_pos']
    fig.add_trace(
        go.Bar(
            x=[ctrl],
            y=[rmse],
            marker=dict(color=colors[ctrl], line=dict(width=1.5, color='white')),
            hovertemplate=f'<b>{ctrl}</b><br>RMSE: {rmse:.2f} mm<br>Energy: {exp2_fault60[ctrl]["energy"]:.2f} Wh<extra></extra>',
            name=ctrl,
            showlegend=False,
            legendgroup=ctrl
        ),
        row=1, col=2
    )

# ────────────────────────────────────────────────────────────────────────
# SUBPLOT 3: Exp 3 - Wind
# ────────────────────────────────────────────────────────────────────────
for i, ctrl in enumerate(controller_order):
    rmse = exp3_wind[ctrl]['rmse_pos']
    fig.add_trace(
        go.Bar(
            x=[ctrl],
            y=[rmse],
            marker=dict(color=colors[ctrl], line=dict(width=1.5, color='white')),
            hovertemplate=f'<b>{ctrl}</b><br>RMSE: {rmse:.2f} mm<br>Energy: {exp3_wind[ctrl]["energy"]:.2f} Wh<extra></extra>',
            name=ctrl,
            showlegend=False,
            legendgroup=ctrl
        ),
        row=2, col=1
    )

# ────────────────────────────────────────────────────────────────────────
# SUBPLOT 4: Exp 4 - Delay (subset, only stable controllers)
# ────────────────────────────────────────────────────────────────────────
for i, ctrl in enumerate(controller_order):
    if ctrl in exp4_delay5ms:
        rmse = exp4_delay5ms[ctrl]['rmse_pos']
        fig.add_trace(
            go.Bar(
                x=[ctrl],
                y=[rmse],
                marker=dict(color=colors[ctrl], line=dict(width=1.5, color='white')),
                hovertemplate=f'<b>{ctrl}</b><br>RMSE: {rmse:.2f} mm<br>Stable @ 5ms<extra></extra>',
                name=ctrl,
                showlegend=False,
                legendgroup=ctrl
            ),
            row=2, col=2
        )

# ────────────────────────────────────────────────────────────────────────
# UPDATE LAYOUT
# ────────────────────────────────────────────────────────────────────────

fig.update_yaxes(title_text="<b>Position RMSE (mm)</b>", row=1, col=1, title_font=dict(size=12, family='Times'))
fig.update_yaxes(title_text="<b>Position RMSE (mm)</b>", row=1, col=2, title_font=dict(size=12, family='Times'))
fig.update_yaxes(title_text="<b>Position RMSE (mm)</b>", row=2, col=1, title_font=dict(size=12, family='Times'))
fig.update_yaxes(title_text="<b>Position RMSE (mm)</b>", row=2, col=2, title_font=dict(size=12, family='Times'))

# X-axis labels
for i in range(1, 5):
    row = (i - 1) // 2 + 1
    col = (i - 1) % 2 + 1
    fig.update_xaxes(tickangle=-45, row=row, col=col)

# Global layout
fig.update_layout(
    title=dict(
        text='<b>Comparative RMSE Performance Across All Experiments</b><br><sub>Neural-AS-NST-STSMC vs. State-of-the-Art Baselines (Octrotor, 6.8 kg)</sub>',
        font=dict(size=16, family='Times New Roman', color='#1f1f1f'),
        x=0.5,
        xanchor='center'
    ),
    height=800,
    width=1400,
    showlegend=True,
    legend=dict(
        orientation='v',
        yanchor='top',
        y=0.99,
        xanchor='left',
        x=1.02,
        bgcolor='rgba(255, 255, 255, 0.8)',
        bordercolor='#cccccc',
        borderwidth=1,
        font=dict(size=10, family='Times')
    ),
    font=dict(family='Times New Roman', size=10, color='#1f1f1f'),
    plot_bgcolor='rgba(240, 240, 245, 0.5)',
    paper_bgcolor='white',
    margin=dict(l=60, r=200, t=120, b=80),
    hovermode='closest'
)

# Add gridlines
fig.update_yaxes(showgrid=True, gridwidth=0.5, gridcolor='rgba(200, 200, 200, 0.3)')

# ============================================================================
# SAVE OUTPUTS
# ============================================================================

print("📊 Generating Figure 6...")

# Save as interactive HTML (best for presentations)
fig.write_html('Fig6_Comparative_RMSE_Interactive.html')
print("✅ Interactive HTML saved: Fig6_Comparative_RMSE_Interactive.html")

# Save as static PNG/PDF (for IEEE submission)
try:
    fig.write_image('Fig6_Comparative_RMSE.pdf', width=1400, height=800)
    print("✅ PDF saved: Fig6_Comparative_RMSE.pdf")
except Exception as e:
    print(f"⚠️  PDF export requires kaleido: pip install kaleido")

try:
    fig.write_image('Fig6_Comparative_RMSE.png', width=1400, height=800, scale=2)
    print("✅ High-res PNG saved: Fig6_Comparative_RMSE.png")
except Exception as e:
    print(f"⚠️  PNG export requires kaleido")

fig.show()