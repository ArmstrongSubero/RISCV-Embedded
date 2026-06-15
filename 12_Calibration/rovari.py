# rovari.py - Rovari Companion App

from rovari import Board
from nicegui import ui

board = Board()

with ui.card().classes('mx-auto mt-12 items-center').style(
    'background: #1E1A2E; border: 1px solid #7C5CBF; width: 240px'
):
    ui.label('ROVARI').style(
        'color: #7C5CBF; font-size: 3rem; font-weight: bold; font-family: monospace')
    lbl = ui.label(
        '--').style('color: white; font-size: 4rem; font-weight: bold; font-family: monospace')
    ui.label('count').style(
        'color: #8B8FA3; font-size: 2.2rem; font-family: monospace')


def poll():
    line = board.readline()
    if line and line.startswith('V:'):
        lbl.set_text(line[2:])


ui.timer(0.1, poll)
ui.run(reload=False, show=True, dark=True)