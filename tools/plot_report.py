import plotly.graph_objects as go
import sys
import time


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python3 plot_reports.py <report_file>")
        exit(1)

    path = sys.argv[1]

    # time, pos, vwPos, vwSpeed, error, power, pout, iout, dout, vout, aout
    reports = []

    TMR_EXP = 2 ** 10

    with open(path, 'r') as file:
        for line in file:
            reports.append([int(num) for num in line.split(",")])

    timestamp = [report[0] / 1000000 for report in reports]
    pos = [report[1] / TMR_EXP for report in reports]
    vwPos = [report[2] / TMR_EXP for report in reports]
    vwSpeed = [report[3] / TMR_EXP for report in reports]
    error = [report[4] / TMR_EXP for report in reports]
    power = [report[5] for report in reports]
    pout = [report[6] for report in reports]
    iout = [report[7] for report in reports]
    dout = [report[8] for report in reports]
    vout = [report[9] for report in reports]
    aout = [report[10] for report in reports]

    err_filtered: list[float] = [0]
    for i in range(1, len(error)):
        err_filtered.append(0.9 * err_filtered[-1] + 0.1 * error[i])

    fig = go.Figure()

    fig.add_trace(go.Scatter(x=timestamp, y=pos, mode='lines', name='pos', line=dict(color='red')))
    fig.add_trace(go.Scatter(x=timestamp, y=vwPos, mode='lines', name='vwPos', line=dict(color='blue')))
    fig.add_trace(go.Scatter(x=timestamp, y=vwSpeed, mode='lines', name='vwSpeed', line=dict(color='green'), yaxis='y2'))
    fig.add_trace(go.Scatter(x=timestamp, y=error, mode='lines', name='error', line=dict(color='purple'), yaxis='y3'))
    fig.add_trace(go.Scatter(x=timestamp, y=power, mode='lines', name='power', line=dict(color='orange'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=pout, mode='lines', name='pout', line=dict(color='brown'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=iout, mode='lines', name='iout', line=dict(color='gray'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=dout, mode='lines', name='dout', line=dict(color='cyan'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=vout, mode='lines', name='vout', line=dict(color='olive'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=aout, mode='lines', name='aout', line=dict(color='pink'), yaxis='y4'))
    fig.add_trace(go.Scatter(x=timestamp, y=err_filtered, mode='lines', name='err_filtered', line=dict(color='black'), yaxis='y3'))

    fig.update_layout(
        title=f'DC Motor Test Report (rendered@{time.strftime("%Y-%m-%d %H:%M:%S")})',
        xaxis=dict(title='time (s)', domain=[0.05, 0.95]),
        yaxis=dict(title='Position (ticks)', tickfont=dict(color='red')),
        yaxis2=dict(title='vwSpeed (ticks/s)', tickfont=dict(color='green'), overlaying='y', side='right', anchor='x'),
        yaxis3=dict(title='error (ticks)', tickfont=dict(color='purple'), overlaying='y', side='left', anchor='free', position=0),
        yaxis4=dict(title='power', tickfont=dict(color='orange'), overlaying='y', side='right', anchor='free', position=1),
        legend=dict(x=0.05, y=1, bgcolor='rgba(255,255,255,0.7)'),
    )

    fig.show()


if __name__ == '__main__':
    main()
