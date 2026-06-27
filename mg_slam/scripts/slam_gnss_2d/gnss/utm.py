from __future__ import annotations

from dataclasses import dataclass

from pyproj import CRS, Transformer


@dataclass(frozen=True)
class UtmTransformer:
    transformer: Transformer
    zone: int
    south: bool


def build_utm_transformer(latitude: float, longitude: float) -> UtmTransformer:
    """緯度経度からUTMゾーンを決め、変換器を生成する。"""
    zone = int((longitude + 180.0) / 6.0) + 1
    south = latitude < 0.0
    crs_utm = CRS.from_dict({'proj': 'utm', 'zone': zone, 'south': south})
    transformer = Transformer.from_crs('EPSG:4326', crs_utm, always_xy=True)
    return UtmTransformer(transformer=transformer, zone=zone, south=south)


def transform_latlon(transformer: UtmTransformer, latitude: float, longitude: float) -> tuple[float, float]:
    """緯度経度をUTM平面座標へ変換する。"""
    return transformer.transformer.transform(longitude, latitude)
