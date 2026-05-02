import { useEffect, useRef, useState, useCallback } from 'react'

export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected' | 'error'

export interface FoxgloveClientHandle {
  status: ConnectionStatus
  subscribe: (topic: string, schemaName: string, onMessage: (data: unknown) => void) => () => void
  callService: (service: string, payload: unknown) => Promise<unknown>
  publish: (topic: string, schemaName: string, data: unknown) => void
}

type ChannelId = number
type SubscriptionId = number

interface ServerChannel {
  id: ChannelId
  topic: string
  encoding: string
  schemaName: string
  schema: string
}

interface ServiceInfo {
  id: number
  requestEncoding: string
  requestSchema: string
}

type CdrPrimitive =
  | 'bool' | 'uint8' | 'int8'
  | 'uint16' | 'int16'
  | 'uint32' | 'int32' | 'float32'
  | 'uint64' | 'int64' | 'float64'
  | 'string'

function parseCdrFields(schema: string): Array<{ type: CdrPrimitive; name: string }> {
  const primitives = new Set<string>([
    'bool', 'uint8', 'int8', 'uint16', 'int16',
    'uint32', 'int32', 'float32', 'uint64', 'int64', 'float64', 'string',
  ])
  const fields: Array<{ type: CdrPrimitive; name: string }> = []
  for (const raw of schema.split('\n')) {
    const line = raw.trim()
    if (!line || line.startsWith('#') || line === '---') continue
    if (line.includes('=')) continue
    const parts = line.split(/\s+/)
    if (parts.length >= 2 && primitives.has(parts[0])) {
      fields.push({ type: parts[0] as CdrPrimitive, name: parts[1] })
    }
  }
  return fields
}

function cdrAlign(pos: number, type: CdrPrimitive): number {
  const alignment: Record<CdrPrimitive, number> = {
    bool: 1, uint8: 1, int8: 1,
    uint16: 2, int16: 2,
    uint32: 4, int32: 4, float32: 4, string: 4,
    uint64: 8, int64: 8, float64: 8,
  }
  const a = alignment[type]
  return a <= 1 ? pos : Math.ceil(pos / a) * a
}

function encodeCDRRequest(payload: Record<string, unknown>, schema: string): Uint8Array {
  const fields = parseCdrFields(schema)
  const chunks: Uint8Array[] = [new Uint8Array([0x00, 0x01, 0x00, 0x00])]
  let pos = 4
  for (const { type, name } of fields) {
    const aligned = cdrAlign(pos, type)
    if (aligned > pos) {
      chunks.push(new Uint8Array(aligned - pos))
      pos = aligned
    }
    const val = payload[name] ?? 0
    const n = typeof val === 'boolean' ? (val ? 1 : 0) : Number(val)
    let b: Uint8Array
    switch (type) {
      case 'bool': case 'uint8': case 'int8':
        b = new Uint8Array([n & 0xff]); break
      case 'uint16': case 'int16':
        b = new Uint8Array(2); new DataView(b.buffer).setUint16(0, n, true); break
      case 'uint32': case 'int32':
        b = new Uint8Array(4); new DataView(b.buffer).setUint32(0, n >>> 0, true); break
      case 'float32':
        b = new Uint8Array(4); new DataView(b.buffer).setFloat32(0, n, true); break
      case 'uint64': case 'int64':
        b = new Uint8Array(8); new DataView(b.buffer).setBigUint64(0, BigInt(Math.trunc(n)), true); break
      case 'float64':
        b = new Uint8Array(8); new DataView(b.buffer).setFloat64(0, n, true); break
      case 'string': {
        const sb = new TextEncoder().encode(String(val ?? ''))
        b = new Uint8Array(4 + sb.byteLength + 1)
        new DataView(b.buffer).setUint32(0, sb.byteLength + 1, true)
        b.set(sb, 4)
        break
      }
    }
    chunks.push(b)
    pos += b.byteLength
  }
  const total = chunks.reduce((s, c) => s + c.byteLength, 0)
  const out = new Uint8Array(total)
  let off = 0
  for (const c of chunks) { out.set(c, off); off += c.byteLength }
  return out
}

const SUBPROTOCOL = ['foxglove.websocket.v1', 'foxglove.sdk.v1']
const RECONNECT_INTERVAL_MS = 3000
const SERVICE_CALL_TIMEOUT_MS = 10000

const MSG_OPCODE_CLIENT_MESSAGE = 0x01
const MSG_OPCODE_SERVICE_CALL_REQUEST = 0x02
const MSG_OPCODE_MESSAGE_DATA = 0x01
const MSG_OPCODE_SERVICE_CALL_RESPONSE = 0x03

function getWsUrl(): string {
  const host = window.location.hostname
  return `ws://${host}:8765/`
}

function normalizeName(name: string): string {
  return name.startsWith('/') ? name.slice(1) : name
}

export function useFoxgloveClient(): FoxgloveClientHandle {
  const [status, setStatus] = useState<ConnectionStatus>('connecting')
  const statusRef = useRef<ConnectionStatus>('connecting')
  const wsRef = useRef<WebSocket | null>(null)
  const channelsByTopicRef = useRef<Map<string, ServerChannel>>(new Map())
  const subscriptionsRef = useRef<Map<SubscriptionId, (data: unknown) => void>>(new Map())
  const clientChIdRef = useRef<Map<string, number>>(new Map())
  const pendingServicesRef = useRef<Map<number, { resolve: (v: unknown) => void; reject: (e: unknown) => void }>>(new Map())
  const serviceCallIdRef = useRef(0)
  const subIdCounterRef = useRef(0)
  const clientChIdCounterRef = useRef(0)
  const servicesByNameRef = useRef<Map<string, ServiceInfo>>(new Map())
  const mountedRef = useRef(false)
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  const updateStatus = useCallback((s: ConnectionStatus) => {
    statusRef.current = s
    setStatus(s)
  }, [])

  const connect = useCallback(() => {
    if (!mountedRef.current) return
    const ws = new WebSocket(getWsUrl(), SUBPROTOCOL)
    ws.binaryType = 'arraybuffer'
    wsRef.current = ws

    ws.onopen = () => updateStatus('connected')
    ws.onerror = () => updateStatus('error')
    ws.onclose = () => {
      updateStatus('disconnected')
      channelsByTopicRef.current.clear()
      clientChIdRef.current.clear()
      servicesByNameRef.current.clear()
      if (mountedRef.current) {
        reconnectTimerRef.current = setTimeout(connect, RECONNECT_INTERVAL_MS)
      }
    }

    ws.onmessage = (event: MessageEvent) => {
      if (typeof event.data === 'string') {
        let msg: Record<string, unknown>
        try {
          msg = JSON.parse(event.data) as Record<string, unknown>
        } catch {
          return
        }
        const op = msg['op'] as string | undefined
        if (op === 'advertise') {
          const channels = msg['channels'] as ServerChannel[]
          for (const ch of channels) {
            channelsByTopicRef.current.set(ch.topic, ch)
            channelsByTopicRef.current.set(normalizeName(ch.topic), ch)
          }
        } else if (op === 'unadvertise') {
          const channelIds = msg['channelIds'] as ChannelId[]
          for (const id of channelIds) {
            for (const [key, ch] of channelsByTopicRef.current) {
              if (ch.id === id) channelsByTopicRef.current.delete(key)
            }
          }
        } else if (op === 'advertiseServices') {
          const services = msg['services'] as Array<{
            id: number
            name: string
            request?: { encoding: string; schema: string }
          }>
          for (const svc of services) {
            const info: ServiceInfo = {
              id: svc.id,
              requestEncoding: svc.request?.encoding ?? 'cdr',
              requestSchema: svc.request?.schema ?? '',
            }
            servicesByNameRef.current.set(svc.name, info)
            servicesByNameRef.current.set(normalizeName(svc.name), info)
          }
        } else if (op === 'unadvertiseServices') {
          const serviceIds = msg['serviceIds'] as number[]
          for (const id of serviceIds) {
            for (const [key, info] of servicesByNameRef.current) {
              if (info.id === id) servicesByNameRef.current.delete(key)
            }
          }
        } else if (op === 'serviceCallFailure') {
          const callId = msg['callId'] as number
          const message = msg['message'] as string
          const pending = pendingServicesRef.current.get(callId)
          if (pending) {
            pendingServicesRef.current.delete(callId)
            pending.reject(new Error(message))
          }
        }
      } else {
        if (!(event.data instanceof ArrayBuffer)) return

        const view = new DataView(event.data)
        const opcode = view.getUint8(0)

        if (opcode === MSG_OPCODE_MESSAGE_DATA) {
          const subId = view.getUint32(1, true) as SubscriptionId
          const handler = subscriptionsRef.current.get(subId)
          if (!handler) return
          const payload = event.data.slice(1 + 4 + 8)
          try {
            handler(JSON.parse(new TextDecoder().decode(payload)))
          } catch {
            handler(payload)
          }
        } else if (opcode === MSG_OPCODE_SERVICE_CALL_RESPONSE) {
          const callId = view.getUint32(1 + 4, true)
          const encLen = view.getUint32(1 + 4 + 4, true)
          const pending = pendingServicesRef.current.get(callId)
          if (!pending) return
          pendingServicesRef.current.delete(callId)
          const payload = event.data.slice(1 + 4 + 4 + 4 + encLen)
          try {
            pending.resolve(JSON.parse(new TextDecoder().decode(payload)))
          } catch {
            pending.resolve(payload)
          }
        }
      }
    }
  }, [updateStatus])

  useEffect(() => {
    mountedRef.current = true
    connect()
    return () => {
      mountedRef.current = false
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current)
        reconnectTimerRef.current = null
      }
      const ws = wsRef.current
      wsRef.current = null
      if (ws) {
        ws.onopen = null
        ws.onerror = null
        ws.onclose = null
        ws.onmessage = null
        if (ws.readyState === WebSocket.CONNECTING) {
          ws.addEventListener('open', () => ws.close())
        } else {
          ws.close()
        }
      }
    }
  }, [connect])

  const subscribe = useCallback(
    (topic: string, _schemaName: string, onMessage: (data: unknown) => void): (() => void) => {
      const ws = wsRef.current
      if (!ws || ws.readyState !== WebSocket.OPEN) return () => {}

      const channel = channelsByTopicRef.current.get(topic)
      if (!channel) return () => {}

      const subId = ++subIdCounterRef.current
      subscriptionsRef.current.set(subId, onMessage)
      ws.send(JSON.stringify({ op: 'subscribe', subscriptions: [{ id: subId, channelId: channel.id }] }))

      return () => {
        subscriptionsRef.current.delete(subId)
        if (wsRef.current?.readyState === WebSocket.OPEN) {
          wsRef.current.send(JSON.stringify({ op: 'unsubscribe', subscriptionIds: [subId] }))
        }
      }
    },
    [],
  )

  const callService = useCallback((service: string, payload: unknown): Promise<unknown> => {
    return new Promise((resolve, reject) => {
      const ws = wsRef.current
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        reject(new Error(`not connected (status: ${statusRef.current})`))
        return
      }

      const serviceInfo = servicesByNameRef.current.get(service)
      if (serviceInfo === undefined) {
        reject(new Error(`service not found: ${service} (status: ${statusRef.current})`))
        return
      }

      const callId = ++serviceCallIdRef.current
      pendingServicesRef.current.set(callId, { resolve, reject })

      const encoding = serviceInfo.requestEncoding
      const encodingBytes = new TextEncoder().encode(encoding)
      const payloadBytes =
        encoding === 'cdr'
          ? encodeCDRRequest(payload as Record<string, unknown>, serviceInfo.requestSchema)
          : new TextEncoder().encode(JSON.stringify(payload))
      const buf = new ArrayBuffer(1 + 4 + 4 + 4 + encodingBytes.byteLength + payloadBytes.byteLength)
      const view = new DataView(buf)
      let offset = 0
      view.setUint8(offset++, MSG_OPCODE_SERVICE_CALL_REQUEST)
      view.setUint32(offset, serviceInfo.id, true); offset += 4
      view.setUint32(offset, callId, true); offset += 4
      view.setUint32(offset, encodingBytes.byteLength, true); offset += 4
      new Uint8Array(buf).set(encodingBytes, offset); offset += encodingBytes.byteLength
      new Uint8Array(buf).set(payloadBytes, offset)
      ws.send(buf)

      setTimeout(() => {
        if (pendingServicesRef.current.has(callId)) {
          pendingServicesRef.current.delete(callId)
          reject(new Error('service call timed out'))
        }
      }, SERVICE_CALL_TIMEOUT_MS)
    })
  }, [])

  const publish = useCallback((topic: string, schemaName: string, data: unknown): void => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) return

    let chId = clientChIdRef.current.get(topic)
    if (chId === undefined) {
      chId = ++clientChIdCounterRef.current
      clientChIdRef.current.set(topic, chId)
      ws.send(JSON.stringify({
        op: 'advertise',
        channels: [{ id: chId, topic, encoding: 'json', schemaName }],
      }))
    }

    const payloadBytes = new TextEncoder().encode(JSON.stringify(data))
    const buf = new ArrayBuffer(1 + 4 + payloadBytes.byteLength)
    const view = new DataView(buf)
    view.setUint8(0, MSG_OPCODE_CLIENT_MESSAGE)
    view.setUint32(1, chId, true)
    new Uint8Array(buf).set(payloadBytes, 5)
    ws.send(buf)
  }, [])

  return { status, subscribe, callService, publish }
}
