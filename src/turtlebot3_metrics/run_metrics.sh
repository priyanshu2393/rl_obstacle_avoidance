#!/bin/bash
# Simple metrics collection script

set -e

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="$PACKAGE_DIR/data"
RESULTS_DIR="$PACKAGE_DIR/results"

mkdir -p "$DATA_DIR"
mkdir -p "$RESULTS_DIR"

export TURTLEBOT3_MODEL=waffle_pi

echo "=========================================="
echo "DWA Performance Metrics Collection"
echo "=========================================="
echo ""

# Generate timestamp
TIMESTAMP=$(date +%s)
METRICS_FILE="$DATA_DIR/metrics_$TIMESTAMP.csv"

echo "[1/4] Launching DWA Navigation with Metrics Recording..."
roslaunch turtlebot3_metrics record_metrics.launch metrics_file:=$METRICS_FILE &
LAUNCH_PID=$!

sleep 5

echo "[2/4] Ready! Set goal in RViz using '2D Nav Goal' button"
echo "[2/4] Press Ctrl+C when done"

# Wait for user interrupt
trap "kill $LAUNCH_PID 2>/dev/null || true" EXIT
wait $LAUNCH_PID 2>/dev/null || true

sleep 2

echo "[3/4] Computing metrics..."
python "$PACKAGE_DIR/scripts/compute_metrics.py" "$METRICS_FILE" \
    -o "$RESULTS_DIR/metrics_$TIMESTAMP.json"

echo "[4/4] Generating plots..."
python "$PACKAGE_DIR/scripts/plot_metrics.py" "$METRICS_FILE" \
    -o "$RESULTS_DIR"

echo ""
echo "=========================================="
echo "✓ METRICS COLLECTION COMPLETE!"
echo "=========================================="
echo ""
echo "Results saved to: $RESULTS_DIR/"
echo ""
echo "Generated files:"
ls -lh "$RESULTS_DIR"
echo ""
echo "View metrics:"
cat "$RESULTS_DIR/metrics_$TIMESTAMP.json"