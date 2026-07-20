"""Mouse interaction helpers attached to the matplotlib figure.

Two behaviours are exposed:

* ``attach_scroll_zoom`` adds a wheel-driven zoom for both 2D and 3D axes,
  centred on the cursor in 2D and on the axis centre in 3D.
* ``attach_splitters`` turns the gaps between the three panels into
  draggable handles so the user can rebalance the layout at runtime
  without leaving the live animation.
"""

from __future__ import annotations

import matplotlib.pyplot as plt


def attach_scroll_zoom(fig, axes_2d, axes_3d) -> None:
    """Connect a scroll-event handler that zooms whichever axis is under the cursor."""

    def on_scroll(event):
        ax = event.inaxes
        if ax is None:
            return
        scale = 1 / 1.2 if event.button == "up" else 1.2
        if ax in axes_3d:
            xlim = ax.get_xlim3d()
            ylim = ax.get_ylim3d()
            zlim = ax.get_zlim3d()
            cx = (xlim[0] + xlim[1]) / 2
            cy = (ylim[0] + ylim[1]) / 2
            cz = (zlim[0] + zlim[1]) / 2
            hx = (xlim[1] - xlim[0]) * scale / 2
            hy = (ylim[1] - ylim[0]) * scale / 2
            hz = (zlim[1] - zlim[0]) * scale / 2
            ax.set_xlim3d(cx - hx, cx + hx)
            ax.set_ylim3d(cy - hy, cy + hy)
            ax.set_zlim3d(cz - hz, cz + hz)
        elif ax in axes_2d:
            xd = event.xdata
            yd = event.ydata
            if xd is None or yd is None:
                return
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            ax.set_xlim(xd - (xd - xlim[0]) * scale, xd + (xlim[1] - xd) * scale)
            ax.set_ylim(yd - (yd - ylim[0]) * scale, yd + (ylim[1] - yd) * scale)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("scroll_event", on_scroll)


def attach_pan(fig, ax) -> None:
    """Left-button drag pans (translates) a single 2D axis. No rotation.

    Pixel-delta implementation: the press records the cursor pixel position
    and the axis limits at that instant; motion converts the pixel delta to a
    data delta using the axis pixel size and those frozen limits, then shifts
    both limits. Working from frozen press-time limits (rather than reading
    ``event.xdata`` live) avoids the feedback jitter that appears when the
    limits change mid-drag. Every callback is gated on ``event.inaxes is ax``
    and the left button, so widgets and other axes keep their own behaviour.
    """
    state = {"px": None, "py": None, "xlim": None, "ylim": None}

    def on_press(event):
        if event.button != 1 or event.inaxes is not ax:
            return
        if event.x is None or event.y is None:
            return
        state["px"], state["py"] = event.x, event.y
        state["xlim"], state["ylim"] = ax.get_xlim(), ax.get_ylim()

    def on_motion(event):
        if state["px"] is None or event.x is None or event.y is None:
            return
        bbox = ax.get_window_extent()
        if bbox.width <= 0 or bbox.height <= 0:
            return
        x0, x1 = state["xlim"]
        y0, y1 = state["ylim"]
        ddx = (event.x - state["px"]) * (x1 - x0) / bbox.width
        ddy = (event.y - state["py"]) * (y1 - y0) / bbox.height
        ax.set_xlim(x0 - ddx, x1 - ddx)
        ax.set_ylim(y0 - ddy, y1 - ddy)
        fig.canvas.draw_idle()

    def on_release(event):
        state["px"] = state["py"] = None

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)


def attach_splitters(
    fig,
    ax_top,
    ax_bl,
    ax_br,
    init_h: float = 0.34,
    init_v: float = 0.5,
    left: float = 0.07,
    right: float = 0.03,
    top: float = 0.05,
    bottom: float = 0.08,
) -> None:
    """Draggable horizontal and vertical dividers between the three panels.

    Positions are stored as figure-relative coordinates in [0, 1]. Clicks
    within ~12 px of a divider start a drag; outside that band the normal
    axis interactions keep working untouched. ``left``, ``right``, ``top``,
    ``bottom`` reserve fixed strips of the figure for adjacent widgets
    (e.g. a configuration sidebar on the right).
    """
    state = {"h": init_h, "v": init_v, "drag": None}
    margins = dict(left=left, right=right, top=top, bottom=bottom, gap_h=0.04, gap_v=0.04)
    band_px = 12

    def apply_layout():
        h, v = state["h"], state["v"]
        l, r, t, b = margins["left"], margins["right"], margins["top"], margins["bottom"]
        gh, gv = margins["gap_h"], margins["gap_v"]
        ax_top.set_position([l, h + gh / 2, 1 - l - r, 1 - h - t - gh / 2])
        ax_bl.set_position([l, b, v - l - gv / 2, h - b - gh / 2])
        ax_br.set_position([v + gv / 2, b, 1 - v - r - gv / 2, h - b - gh / 2])
        h_line.set_ydata([h, h])
        v_line.set_xdata([v, v])
        v_line.set_ydata([0, h])

    h_line = plt.Line2D([0, 1], [init_h, init_h], color="0.6", lw=1.5, alpha=0.6)
    v_line = plt.Line2D([init_v, init_v], [0, init_h], color="0.6", lw=1.5, alpha=0.6)
    fig.add_artist(h_line)
    fig.add_artist(v_line)
    apply_layout()

    def _to_fig(event):
        if event.x is None or event.y is None:
            return None, None
        inv = fig.transFigure.inverted()
        fx, fy = inv.transform((event.x, event.y))
        return fx, fy

    def _band_to_fig():
        bbox = fig.get_window_extent()
        return band_px / bbox.height, band_px / bbox.width

    def on_press(event):
        if event.button != 1:
            return
        fx, fy = _to_fig(event)
        if fx is None:
            return
        band_y, band_x = _band_to_fig()
        if abs(fy - state["h"]) < band_y:
            state["drag"] = "h"
        elif fy < state["h"] and abs(fx - state["v"]) < band_x:
            state["drag"] = "v"

    def on_motion(event):
        if state["drag"] is None:
            return
        fx, fy = _to_fig(event)
        if fx is None:
            return
        if state["drag"] == "h":
            state["h"] = max(bottom + 0.05, min(1 - top - 0.05, fy))
        elif state["drag"] == "v":
            state["v"] = max(left + 0.05, min(1 - right - 0.05, fx))
        apply_layout()
        fig.canvas.draw_idle()

    def on_release(event):
        state["drag"] = None

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
