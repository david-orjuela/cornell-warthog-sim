#!/usr/bin/python
# -*- encoding: utf-8 -*-

import torch
import torch.nn as nn

from .bisenetv1 import BiSeNetV1
from .attn_blocks import ECALayer, CoordinateAttention


class BiSeNetV1_ECA(BiSeNetV1):
    """BiSeNetV1 with ECA applied to SpatialPath output and fused features."""

    def __init__(self, n_classes, aux_mode='train', *args, **kwargs):
        super().__init__(n_classes, aux_mode, *args, **kwargs)
        self.eca_sp = ECALayer(128)
        self.eca_fuse = ECALayer(256)

    def forward(self, x):
        feat_cp8, feat_cp16 = self.cp(x)
        feat_sp = self.sp(x)
        feat_sp = self.eca_sp(feat_sp)
        feat_fuse = self.ffm(feat_sp, feat_cp8)
        feat_fuse = self.eca_fuse(feat_fuse)

        feat_out = self.conv_out(feat_fuse)
        if self.aux_mode == 'train':
            feat_out16 = self.conv_out16(feat_cp8)
            feat_out32 = self.conv_out32(feat_cp16)
            return feat_out, feat_out16, feat_out32
        elif self.aux_mode == 'eval':
            return feat_out,
        elif self.aux_mode == 'pred':
            feat_out = self.conv_out(feat_fuse)
            feat_out = feat_out.argmax(dim=1)
            return feat_out
        else:
            raise NotImplementedError


class BiSeNetV1_CA(BiSeNetV1):
    """BiSeNetV1 with Coordinate Attention on CP features and fused features."""

    def __init__(self, n_classes, aux_mode='train', *args, **kwargs):
        super().__init__(n_classes, aux_mode, *args, **kwargs)
        self.ca_cp8 = CoordinateAttention(128, 128)
        self.ca_cp16 = CoordinateAttention(128, 128)
        self.ca_fuse = CoordinateAttention(256, 256)

    def forward(self, x):
        feat_cp8, feat_cp16 = self.cp(x)
        feat_cp8 = self.ca_cp8(feat_cp8)
        feat_cp16 = self.ca_cp16(feat_cp16)
        feat_sp = self.sp(x)
        feat_fuse = self.ffm(feat_sp, feat_cp8)
        feat_fuse = self.ca_fuse(feat_fuse)

        feat_out = self.conv_out(feat_fuse)
        if self.aux_mode == 'train':
            feat_out16 = self.conv_out16(feat_cp8)
            feat_out32 = self.conv_out32(feat_cp16)
            return feat_out, feat_out16, feat_out32
        elif self.aux_mode == 'eval':
            return feat_out,
        elif self.aux_mode == 'pred':
            feat_out = self.conv_out(feat_fuse)
            feat_out = feat_out.argmax(dim=1)
            return feat_out
        else:
            raise NotImplementedError
