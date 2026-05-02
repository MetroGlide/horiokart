import { useEffect, useRef, useState, useCallback } from 'react'
import {
  FoxgloveClient,
  Channel,
  SubscriptionId,
} from '@foxglove/ws-protocol'

export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected' | 'error'

export interface FoxgloveClientHandle {
  status: ConnectionStatus
  subscribe: (topic: string, schemaName: string, onMessage: (data: unknown) => void) => () => void
  callService: (service: string, payload: unknown) => Promise<unknown>
  publish: (topic: string, schemaName: string, data: unknown) => void
}

const RECONNECT_INTERVAL_MS = 3000

function getWsUrl(): string {
  const host = window.location.hostname
  return `ws://${host}:8765`
}

export function useFoxgloveClient(): FoxgloveClientHandle {
  const [status, setStatus] = useState<ConnectionStatus>('connecting')
  const clientRef = useRef<FoxgloveClient | null>(null)
  const channelsByTopicRef = useRef<Map<string, Channel>>(new Map())
  const subscriptionsRef = useRef<Map<SubscriptionId, (data: unknown) => void>>(new Map())
  // advertise() returns ClientChannelId (number)
  const advertiseRef = useRef<Map<string, number>>(new Map())
  const pendingServicesRef = useRef<Map<number, { resolve: (v: unknown) => void; reject: (e: unknown) => void }>>(new Map())
  const serviceCallIdRef = useRef(0)
  // service name → serviceId (number), populated via 'advertiseServices' event
  const servicesByNameRef = useRef<Map<string, number>>(new Map())
  const mountedRef = useRef(false)
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  const connect = useCallback(() => {
    const url = getWsUrl()
    const client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
    })
    clientRef.current = client

    client.on('open', () => setStatus('connected'))
    client.on('error', () => setStatus('error'))
    client.on('close', () => {
      setStatus('disconnected')
      channelsByTopicRef.current.clear()
      advertiseRef.current.clear()
      servicesByNameRef.current.clear()
      if (mountedRef.current) {
        reconnectTimerRef.current = setTimeout(connect, RECONNECT_INTERVAL_MS)
      }
    })

    client.on('advertise', (channels) => {
      for (const ch of channels) {
        channelsByTopicRef.current.set(ch.topic, ch)
      }
    })

    client.on('unadvertise', (channelIds) => {
      for (const id of channelIds) {
        for (const [topic, ch] of channelsByTopicRef.current) {
          if (ch.id === id) channelsByTopicRef.current.delete(topic)
        }
      }
    })

    client.on('advertiseServices', (services) => {
      for (const svc of services) {
        // 先頭の / を除いた名前でも引ける様に正規化して保存
        const normalized = svc.name.startsWith('/') ? svc.name.slice(1) : svc.name
        servicesByNameRef.current.set(normalized, svc.id)
        servicesByNameRef.current.set(svc.name, svc.id)
      }
    })

    client.on('message', ({ subscriptionId, data }) => {
      const handler = subscriptionsRef.current.get(subscriptionId)
      if (!handler) return
      try {
        // data is DataView; TextDecoder accepts ArrayBufferView directly
        const text = new TextDecoder().decode(data)
        handler(JSON.parse(text))
      } catch {
        handler(data)
      }
    })

    client.on('serviceCallResponse', (response) => {
      const pending = pendingServicesRef.current.get(response.callId)
      if (!pending) return
      pendingServicesRef.current.delete(response.callId)
      try {
        const text = new TextDecoder().decode(response.data)
        pending.resolve(JSON.parse(text))
      } catch {
        pending.resolve(response.data)
      }
    })
  }, [])

  useEffect(() => {
    mountedRef.current = true
    connect()
    return () => {
      mountedRef.current = false
      if (reconnectTimerRef.current) {
        clearTimeout(reconnectTimerRef.current)
        reconnectTimerRef.current = null
      }
      clientRef.current?.close()
      clientRef.current = null
    }
  }, [connect])

  const subscribe = useCallback(
    (topic: string, _schemaName: string, onMessage: (data: unknown) => void): (() => void) => {
      const client = clientRef.current
      if (!client) return () => {}

      const channel = channelsByTopicRef.current.get(topic)
      if (!channel) return () => {}

      const subId = client.subscribe(channel.id)
      subscriptionsRef.current.set(subId, onMessage)

      return () => {
        subscriptionsRef.current.delete(subId)
        client.unsubscribe(subId)
      }
    },
    [],
  )

  const callService = useCallback((service: string, payload: unknown): Promise<unknown> => {
    return new Promise((resolve, reject) => {
      const client = clientRef.current
      if (!client) { reject(new Error('not connected')); return }

      const serviceId = servicesByNameRef.current.get(service)
      if (serviceId === undefined) { reject(new Error(`service not found: ${service}`)); return }

      const callId = ++serviceCallIdRef.current
      pendingServicesRef.current.set(callId, { resolve, reject })

      const encoded = new TextEncoder().encode(JSON.stringify(payload))
      client.sendServiceCallRequest({
        callId,
        serviceId,
        encoding: 'json',
        data: new DataView(encoded.buffer),
      })

      setTimeout(() => {
        if (pendingServicesRef.current.has(callId)) {
          pendingServicesRef.current.delete(callId)
          reject(new Error('service call timed out'))
        }
      }, 10000)
    })
  }, [])

  const publish = useCallback((topic: string, schemaName: string, data: unknown): void => {
    const client = clientRef.current
    if (!client) return

    let adChId = advertiseRef.current.get(topic)
    if (adChId === undefined) {
      adChId = client.advertise({ topic, encoding: 'json', schemaName })
      advertiseRef.current.set(topic, adChId)
    }
    const encoded = new TextEncoder().encode(JSON.stringify(data))
    client.sendMessage(adChId, encoded)
  }, [])

  return { status, subscribe, callService, publish }
}
