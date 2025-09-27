# data-model.md

## Entities and Schemas

### PoseGraph (JSON schema)

```
{
  "type": "object",
  "properties": {
    "nodes": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "id": {"type":"integer"},
          "t": {"type":"string","format":"date-time"},
          "pose": {"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},
          "cov": {"type":"array","items":{"type":"number"}}
        },
        "required":["id","t","pose"]
      }
    },
    "edges": {
      "type":"array",
      "items":{
        "type":"object",
        "properties":{
          "from":{"type":"integer"},
          "to":{"type":"integer"},
          "type":{"type":"string"},
          "meas":{"type":"array","items":{"type":"number"}},
          "info":{"type":"array","items":{"type":"number"}}
        },
        "required":["from","to","type","meas"]
      }
    }
  }
}
```

### GNSSObservation (CSV / JSON)

- Fields: timestamp, lat, lon, alt, fix_type, hdop, vdop, covariance_XX, covariance_XY, covariance_YY

### AlignedMap

- Fields: map.pgm, map.yaml metadata: origin_utm: {easting, northing, zone}, resolution, width, height
