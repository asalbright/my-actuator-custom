import os
import pandas as pd
from plotly.subplots import make_subplots
import plotly.graph_objects as go

script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(script_dir, "data")

# List of (filename, y-label, plot title)
csv_specs = [
    ("current_a2_events.csv", "Current (A)", "A2 Current"),
    ("current_a3_events.csv", "Current (A)", "A3 Current"),
    ("shaft_angles_a2_events.csv", "Shaft Angle (deg)", "A2 Shaft Angle"),
    ("shaft_angles_a3_events.csv", "Shaft Angle (deg)", "A3 Shaft Angle"),
    ("shaft_speeds_a2_events.csv", "Shaft Speed (deg/s)", "A2 Shaft Speed"),
    ("shaft_speeds_a3_events.csv", "Shaft Speed (deg/s)", "A3 Shaft Speed"),
    ("tip_x_target_events.csv", "Tip X Target", "Tip X Target"),
    ("a2_target_angles.csv", "A2 Target Angle (deg)", "A2 Target Angle"),
    ("a3_target_angles.csv", "A3 Target Angle (deg)", "A3 Target Angle"),
    ("connor_targets.csv", "Connor X Target", "Connor X Target"),
    ("x_target_differents.csv", "X Target Difference", "X Target Difference"),
    ("tip_x_position_events.csv", "Tip X Position", "Tip X Position"),
    ("position_error_events.csv", "Position Error", "Position Error"),
    ("raw_cg.csv", "Raw CG Value", "Raw CG Value"),
    # Add more CSVs here as you add more event logs
]

# Load available, non-empty CSVs
plot_data = []
for fname, ylabel, title in csv_specs:
    fpath = os.path.join(data_dir, fname)
    if os.path.exists(fpath) and os.path.getsize(fpath) > 0:
        try:
            df = pd.read_csv(fpath)
            if not df.empty:
                plot_data.append((df, ylabel, title))
        except Exception as e:
            print(f"Could not read {fname}: {e}")

n_rows = len(plot_data)
if n_rows == 0:
    print("No data to plot.")
    exit()

fig = make_subplots(
    rows=n_rows,
    cols=1,
    shared_xaxes=True,
    vertical_spacing=0.05,
    subplot_titles=[title for _, _, title in plot_data],
)

for idx, (df, ylabel, title) in enumerate(plot_data):
    row = idx + 1
    fig.add_trace(
        go.Scatter(
            x=df[df.columns[0]],  # Time (s)
            y=df[df.columns[1]],  # Value
            mode="lines+markers",
            name=title,
        ),
        row=row,
        col=1,
    )
    fig.update_yaxes(title_text=ylabel, row=row, col=1)

fig.update_layout(
    title="Planar Robot Data Plots",
    xaxis=dict(title="Time (s)", tickangle=45),
    legend_title="Legend",
    hovermode="x unified",
    template="plotly_white",
    height=300 * n_rows,
)

fig.show()


def overlay_csvs(csv_list, data_dir, title="Overlayed Plot"):
    """
    Overlay multiple CSVs (from your event logs) on a single subplot.
    csv_list: list of (filename, y-label, legend name) tuples
    data_dir: directory where CSVs are stored
    title: plot title
    """
    import plotly.graph_objects as go
    import pandas as pd
    import os

    fig = go.Figure()
    for fname, ylabel, legend in csv_list:
        fpath = os.path.join(data_dir, fname)
        if os.path.exists(fpath) and os.path.getsize(fpath) > 0:
            try:
                df = pd.read_csv(fpath)
                if not df.empty:
                    fig.add_trace(
                        go.Scatter(
                            x=df[df.columns[0]],
                            y=df[df.columns[1]],
                            mode="lines+markers",
                            name=legend,
                        )
                    )
            except Exception as e:
                print(f"Could not read {fname}: {e}")

    fig.update_layout(
        title=title,
        xaxis=dict(title="Time (s)", tickangle=45),
        yaxis=dict(title="Value"),
        legend_title="Legend",
        hovermode="x unified",
        template="plotly_white",
        height=500,
    )
    fig.show()


overlay_csvs(
    [
        ("connor_targets.csv", "Connor X Target", "Connor X Target"),
        ("tip_x_position_events.csv", "Tip X Position", "Tip X Position"),
        ("raw_cg.csv", "Raw CG Value", "Raw CG Value"),
    ],
    data_dir,
    title="Tip X Position & Target Overlay",
)
