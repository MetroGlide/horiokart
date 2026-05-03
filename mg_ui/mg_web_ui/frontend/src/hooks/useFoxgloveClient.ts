import { parse } from '@foxglove/rosmsg'
import { MessageReader, MessageWriter } from '@foxglove/rosmsg2-serialization'
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

interface AdvertisedSchema {
  encoding: string
  schemaName: string
  schema: string
}

interface ServiceInfo {
  id: number
  request?: AdvertisedSchema
  response?: AdvertisedSchema
}

const CLIENT_CHANNEL_SCHEMAS: Record<string, AdvertisedSchema> = {
  'mg_msgs/msg/PauseRequest': {
    encoding: 'cdr',
    schemaName: 'mg_msgs/msg/PauseRequest',
    schema: 'string requester_id\nbool active\nfloat32 heartbeat_period_s\nstring reason',
  },
  'std_msgs/msg/Int16': {
    encoding: 'cdr',
    schemaName: 'std_msgs/msg/Int16',
    schema: 'int16 data',
  },
  'std_msgs/msg/String': {
    encoding: 'cdr',
    schemaName: 'std_msgs/msg/String',
    schema: 'string data',
  },
  'geometry_msgs/msg/PoseWithCovarianceStamped': {
    encoding: 'cdr',
    schemaName: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    schema: `std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w`,
  },
}

function getMessageReader(cache: Map<string, MessageReader>, schema: AdvertisedSchema): MessageReader | undefined {
  const key = `${schema.schemaName}:${schema.schema}`
  const cached = cache.get(key)
  if (cached) return cached

  try {
    const reader = new MessageReader(parse(schema.schema, { ros2: true }))
    cache.set(key, reader)
    return reader
  } catch {
    return undefined
  }
}

function getMessageWriter(cache: Map<string, MessageWriter>, schema: AdvertisedSchema): MessageWriter | undefined {
  const key = `${schema.schemaName}:${schema.schema}`
  const cached = cache.get(key)
  if (cached) return cached

  try {
    const writer = new MessageWriter(parse(schema.schema, { ros2: true }))
    cache.set(key, writer)
    return writer
  } catch {
    return undefined
  }
}

function decodePayload(
  payload: ArrayBuffer,
  schema: AdvertisedSchema | undefined,
  readerCache: Map<string, MessageReader>,
): unknown {
  if (!schema) {
    return payload
  }

  if (schema.encoding === 'json') {
    try {
      return JSON.parse(new TextDecoder().decode(payload))
    } catch {
      return payload
    }
  }

  if (schema.encoding !== 'cdr') {
    return payload
  }

  const reader = getMessageReader(readerCache, schema)
  if (!reader) {
    return payload
  }

  try {
    return reader.readMessage(new Uint8Array(payload))
  } catch {
    return payload
  }
}

function encodePayload(
  data: unknown,
  schema: AdvertisedSchema | undefined,
  writerCache: Map<string, MessageWriter>,
): Uint8Array {
  if (!schema || schema.encoding === 'json') {
    return new TextEncoder().encode(JSON.stringify(data))
  }

  const writer = getMessageWriter(writerCache, schema)
  if (!writer) {
    return new TextEncoder().encode(JSON.stringify(data))
  }

  try {
    return writer.writeMessage(data as Record<string, unknown>)
  } catch {
    return new TextEncoder().encode(JSON.stringify(data))
  }
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
  const subscriptionChannelsRef = useRef<Map<SubscriptionId, ServerChannel>>(new Map())
  const messageReadersRef = useRef<Map<string, MessageReader>>(new Map())
  const messageWritersRef = useRef<Map<string, MessageWriter>>(new Map())
  const clientChIdRef = useRef<Map<string, number>>(new Map())
  const pendingServicesRef = useRef<Map<number, {
    resolve: (v: unknown) => void
    reject: (e: unknown) => void
    response?: AdvertisedSchema
  }>>(new Map())
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
      subscriptionChannelsRef.current.clear()
      messageReadersRef.current.clear()
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
            request?: { encoding: string; schemaName: string; schema: string }
            response?: { encoding: string; schemaName: string; schema: string }
          }>
          for (const svc of services) {
            const info: ServiceInfo = {
              id: svc.id,
              request: svc.request,
              response: svc.response,
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
          const channel = subscriptionChannelsRef.current.get(subId)
          if (!handler) return
          const payload = event.data.slice(1 + 4 + 8)
          handler(decodePayload(payload, channel, messageReadersRef.current))
        } else if (opcode === MSG_OPCODE_SERVICE_CALL_RESPONSE) {
          const callId = view.getUint32(1 + 4, true)
          const encLen = view.getUint32(1 + 4 + 4, true)
          const pending = pendingServicesRef.current.get(callId)
          if (!pending) return
          pendingServicesRef.current.delete(callId)
          const encoding = new TextDecoder().decode(event.data.slice(1 + 4 + 4 + 4, 1 + 4 + 4 + 4 + encLen))
          const payload = event.data.slice(1 + 4 + 4 + 4 + encLen)
          pending.resolve(
            decodePayload(
              payload,
              pending.response ? { ...pending.response, encoding } : undefined,
              messageReadersRef.current,
            ),
          )
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
      subscriptionChannelsRef.current.set(subId, channel)
      ws.send(JSON.stringify({ op: 'subscribe', subscriptions: [{ id: subId, channelId: channel.id }] }))

      return () => {
        subscriptionsRef.current.delete(subId)
        subscriptionChannelsRef.current.delete(subId)
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
      pendingServicesRef.current.set(callId, {
        resolve,
        reject,
        response: serviceInfo.response,
      })

      const requestSchema = serviceInfo.request
      const encoding = requestSchema?.encoding ?? 'json'
      const encodingBytes = new TextEncoder().encode(encoding)
      const payloadBytes = encodePayload(payload, requestSchema, messageWritersRef.current)
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

    const schema = CLIENT_CHANNEL_SCHEMAS[schemaName]
    const encoding = schema?.encoding ?? 'json'
    const payloadBytes = encodePayload(data, schema, messageWritersRef.current)

    let chId = clientChIdRef.current.get(topic)
    if (chId === undefined) {
      chId = ++clientChIdCounterRef.current
      clientChIdRef.current.set(topic, chId)
      ws.send(JSON.stringify({
        op: 'advertise',
        channels: [{ id: chId, topic, encoding, schemaName }],
      }))
    }

    const buf = new ArrayBuffer(1 + 4 + payloadBytes.byteLength)
    const view = new DataView(buf)
    view.setUint8(0, MSG_OPCODE_CLIENT_MESSAGE)
    view.setUint32(1, chId, true)
    new Uint8Array(buf).set(payloadBytes, 5)
    ws.send(buf)
  }, [])

  return { status, subscribe, callService, publish }
}
