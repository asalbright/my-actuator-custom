import pandas as pd
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# Load the CSV file into a DataFrame
data = pd.read_csv("motor_data.csv")

# Extract columns
time = data["Time (s)"]
current = data["Current (A)"]
shaft_speed = data["Shaft Speed (rpm)"]
shaft_angle = data["Shaft Angle (deg)"]
position = data["Position (deg)"]

# Create subplots: 4 rows, 1 column
fig = make_subplots(rows=4, cols=1, shared_xaxes=True, vertical_spacing=0.1,  # Increased vertical spacing
                    subplot_titles=("Motor Current", "Shaft Speed", "Shaft Angle", "Input Position"))

# Add Current to the first subplot
fig.add_trace(go.Scatter(x=time, y=current, mode='lines', name='Current (A)', line=dict(color='blue')), row=1, col=1)

# Add Shaft Speed to the second subplot
fig.add_trace(go.Scatter(x=time, y=shaft_speed, mode='lines', name='Shaft Speed (rpm)', line=dict(color='orange')), row=2, col=1)

# Add Shaft Angle to the third subplot
fig.add_trace(go.Scatter(x=time, y=shaft_angle, mode='lines', name='Shaft Angle (deg)', line=dict(color='green')), row=3, col=1)
fig.add_trace(go.Scatter(x=time, y=position, mode='lines', name='Position (deg)', line=dict(color='red')), row=3, col=1)

# Add Position to the fourth subplot

# Update layout for better interactivity
fig.update_layout(
    title="Position Tuning Plots",
    xaxis=dict(title="Time (s)", tickangle=45),  # Rotate x-axis labels by 45 degrees
    yaxis_title="Values",
    legend_title="Legend",
    hovermode="x unified",
    template="plotly_white",
    height=900  # Adjust height for better spacing
)

# Show the interactive plot
fig.show()