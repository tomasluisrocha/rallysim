"""The driver's inputs as plain bar charts, shared by topdown.py and view3d.py: the steer as a horizontal bar (left
of zero when steering left) and every other input as a vertical bar, the share of its range."""
from rallytools.overhead import COLOURS, Channel  # noqa: F401  (they live in the library)


class InputBars:
    """The inputs as vertical bars on `ax_bars`, and the steer as a horizontal bar on `ax_steer`, in degrees, with the
    axis turned round so steering left (positive) draws to the left. Built once, redrawn with update(i)."""

    def __init__(self, ax_bars, ax_steer, steer, others):
        self.ax_steer, self.steer, self.others = ax_steer, steer, others
        self.bars = list(ax_bars.bar([c.name for c in others], [0.0] * len(others), width=0.6,
                                     color=[c.colour for c in others], edgecolor="black"))
        self.texts = [ax_bars.text(k, 0, "", ha="center", fontsize=8) for k in range(len(others))]
        ax_bars.set_ylim(-1.25 if any(c.low < 0 for c in others) else 0, 1.25)
        ax_bars.axhline(0, color="black", lw=0.8)
        ax_bars.grid(True, axis="y", alpha=0.3)
        ax_bars.tick_params(labelsize=8)
        ax_bars.set_title("driver inputs (share of range)", fontsize=9)
        self.steer_bar = None
        if steer is not None:
            top = max(-steer.low, steer.high)
            self.steer_bar = ax_steer.barh([0], [0.0], height=0.6, color=steer.colour, edgecolor="black")[0]
            ax_steer.set_xlim(top, -top)
            ax_steer.set_ylim(-0.6, 0.6)
            ax_steer.axvline(0, color="black", lw=0.8)
            ax_steer.set_yticks([])
            ax_steer.grid(True, axis="x", alpha=0.3)
            ax_steer.tick_params(labelsize=8)
            ax_steer.set_xlabel("steer (deg), left +", fontsize=8)
        elif ax_steer is not None:
            ax_steer.set_axis_off()

    def update(self, i):
        for bar, text, c in zip(self.bars, self.texts, self.others):
            share = c.share(i)
            bar.set_height(share)
            bar.set_facecolor(c.below if c.below and share < 0 else c.colour)
            text.set_position((bar.get_x() + bar.get_width() / 2, share + (0.05 if share >= 0 else -0.05)))
            text.set_va("bottom" if share >= 0 else "top")
            text.set_text(c.text(i))
        if self.steer_bar is not None:
            self.steer_bar.set_width(float(self.steer.values[i]))
            self.ax_steer.set_title(f"steer {self.steer.text(i)}", fontsize=8)
