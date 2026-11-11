"""Colours, fonts and the Qt stylesheet of the AirMouse ground station."""

from __future__ import annotations

try:
    from PyQt5 import QtGui
except ImportError:                                   # ROS installs ship python_qt_binding
    from python_qt_binding import QtGui  # type: ignore

BG = "#070b14"
SURFACE = "#0d1526"
SURFACE2 = "#111c31"
RAISED = "#16233a"
BORDER = "#1e2b44"
TEXT = "#e6edf6"
MUTED = "#8a9ab0"
DIM = "#5b6b88"
ACCENT = "#22d3ee"
GREEN = "#22c55e"
AMBER = "#f59e0b"
RED = "#ef4444"
VIOLET = "#a78bfa"
PINK = "#e879f9"
YELLOW = "#facc15"
GREY = "#7c8a9e"

FAMILIES = ["Inter", "Segoe UI", "Ubuntu", "Noto Sans", "DejaVu Sans"]
MONO = ["JetBrains Mono", "Cascadia Mono", "DejaVu Sans Mono", "Consolas"]
_WEIGHTS = {400: QtGui.QFont.Normal, 500: QtGui.QFont.Medium, 600: QtGui.QFont.DemiBold,
            700: QtGui.QFont.Bold, 800: QtGui.QFont.ExtraBold}

STATUS_COLOURS = {"pass": GREEN, "warn": AMBER, "fail": RED, "na": GREY}
EVENT_COLOURS = {"survivor": GREEN, "warn": AMBER, "alert": RED, "ok": ACCENT, "brain": PINK, "info": MUTED}


def font(px: float, weight: int = 400, mono: bool = False) -> QtGui.QFont:
    f = QtGui.QFont()
    f.setFamilies(MONO if mono else FAMILIES)
    f.setPixelSize(int(round(px)))
    f.setWeight(_WEIGHTS.get(weight, QtGui.QFont.Normal))
    return f


def colour(hexstr: str, alpha: int = 255) -> QtGui.QColor:
    c = QtGui.QColor(hexstr)
    c.setAlpha(alpha)
    return c


STYLE = f"""
QMainWindow, QWidget {{ background:{BG}; color:{TEXT}; font-family:"Inter","Segoe UI","Ubuntu","Noto Sans","DejaVu Sans";
                       font-size:13px; }}
QFrame#topbar {{ background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 #0e1a30, stop:1 #0a1324);
                border-bottom:1px solid {BORDER}; }}
QFrame#rail {{ background:#0a1120; border-right:1px solid {BORDER}; }}
QToolButton#navButton {{ background:transparent; color:{MUTED}; border:none; border-radius:12px;
                         padding:10px 4px; font-size:11px; font-weight:600; }}
QToolButton#navButton:hover {{ background:{SURFACE2}; color:{TEXT}; }}
QToolButton#navButton:checked {{ background:#0f2a3a; color:{ACCENT}; }}
QFrame#card {{ background:{SURFACE}; border:1px solid {BORDER}; border-radius:14px; }}
QLabel#cardTitle {{ color:{MUTED}; font-size:11px; font-weight:700; letter-spacing:1.5px; background:transparent; }}
QLabel {{ background:transparent; }}
QPushButton {{ background:{RAISED}; color:{TEXT}; border:1px solid {BORDER}; border-radius:10px;
              padding:8px 16px; font-weight:600; }}
QPushButton:hover {{ background:#1c2c48; border-color:#2b3f63; }}
QPushButton:pressed {{ background:#132038; }}
QPushButton:disabled {{ color:{DIM}; background:{SURFACE}; }}
QPushButton#primary {{ background:#0e7490; border:1px solid #0e7490; color:white; }}
QPushButton#primary:hover {{ background:#0891b2; }}
QPushButton#seg {{ padding:6px 12px; border-radius:8px; }}
QPushButton#seg:checked {{ background:#0f2a3a; color:{ACCENT}; border-color:{ACCENT}; }}
QTableWidget {{ background:{SURFACE}; alternate-background-color:#0f192c; color:{TEXT}; border:none;
               font-size:13px; selection-background-color:#163352; selection-color:{TEXT}; gridline-color:{BORDER}; }}
QHeaderView::section {{ background:{SURFACE2}; color:{MUTED}; padding:8px 10px; border:none;
                       border-bottom:1px solid {BORDER}; font-weight:700; font-size:11px; }}
QTableCornerButton::section {{ background:{SURFACE2}; border:none; }}
QListWidget {{ background:{SURFACE}; border:none; outline:none; }}
QListWidget::item {{ border-radius:10px; padding:10px; margin:3px 4px; color:{TEXT}; }}
QListWidget::item:hover {{ background:{SURFACE2}; }}
QListWidget::item:selected {{ background:#0f2a3a; color:{TEXT}; border:1px solid {ACCENT}; }}
QCheckBox {{ color:#cbd5e1; spacing:6px; background:transparent; }}
QCheckBox::indicator {{ width:15px; height:15px; border-radius:4px; border:1px solid #475569; background:#0b1220; }}
QCheckBox::indicator:checked {{ background:{ACCENT}; border:1px solid {ACCENT}; }}
QScrollArea {{ background:transparent; border:none; }}
QScrollBar:vertical {{ background:transparent; width:10px; }}
QScrollBar::handle:vertical {{ background:#24324a; border-radius:5px; min-height:24px; }}
QScrollBar:horizontal {{ background:transparent; height:10px; }}
QScrollBar::handle:horizontal {{ background:#24324a; border-radius:5px; min-width:24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width:0px; height:0px; }}
QStatusBar {{ background:#0a1120; color:{MUTED}; font-size:12px; border-top:1px solid {BORDER}; }}
QToolTip {{ background:{RAISED}; color:{TEXT}; border:1px solid {BORDER}; padding:6px; }}
"""
