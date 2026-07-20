import torch
from torch import nn
import math


class ECALayer(nn.Module):
    """Efficient Channel Attention (from pytorch-attention/attention_mechanisms/eca.py)"""

    def __init__(self, channels, gamma=2, b=1):
        super().__init__()
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        t = int(abs((math.log(channels, 2) + b) / gamma))
        k = t if t % 2 else t + 1
        self.conv = nn.Conv1d(1, 1, kernel_size=k, padding=(k - 1) // 2, bias=False)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        y = self.avgpool(x)
        y = self.conv(y.squeeze(-1).transpose(-1, -2)).transpose(-1, -2).unsqueeze(-1)
        y = self.sigmoid(y)
        return x * y.expand_as(x)


class CoordinateAttention(nn.Module):
    """Coordinate Attention (from pytorch-attention/attention_mechanisms/coordatten.py)"""

    def __init__(self, in_dim, out_dim, reduction=32):
        super().__init__()
        self.pool_h = nn.AdaptiveAvgPool2d((None, 1))
        self.pool_w = nn.AdaptiveAvgPool2d((1, None))
        hidden_dim = max(8, in_dim // reduction)
        self.conv1 = nn.Conv2d(in_dim, hidden_dim, kernel_size=1, stride=1, padding=0)
        self.bn1 = nn.BatchNorm2d(hidden_dim)
        self.act = nn.ReLU(inplace=True)
        self.conv_h = nn.Conv2d(hidden_dim, out_dim, kernel_size=1, stride=1, padding=0)
        self.conv_w = nn.Conv2d(hidden_dim, out_dim, kernel_size=1, stride=1, padding=0)

    def forward(self, x):
        identity = x
        b, c, h, w = x.shape
        x_h = self.pool_h(x)
        x_w = self.pool_w(x).transpose(-1, -2)
        y = torch.cat([x_h, x_w], dim=2)
        y = self.conv1(y)
        y = self.bn1(y)
        y = self.act(y)
        x_h, x_w = torch.split(y, [h, w], dim=2)
        x_w = x_w.transpose(-1, -2)
        a_h = self.conv_h(x_h)
        a_w = self.conv_w(x_w)
        # Bound attention to [0, 1] to stabilize training in FP16
        a_h = torch.sigmoid(a_h)
        a_w = torch.sigmoid(a_w)
        out = identity * a_h * a_w
        return out
