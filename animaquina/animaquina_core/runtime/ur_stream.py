# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core — UR Dynamic Sync URScript generator
#
# Generates a URScript program that mirrors KUKA's mq_stream.src:
# - Uses RTDE input registers as a ring buffer for waypoints
# - Uses RTDE output registers for read-index and done-id feedback
# - Blender writes waypoints to input registers, reads progress from output registers
# - The URScript loops reading the ring buffer and executing movel() with blend
#
# Register layout (ring_buffer_size=6, 6 floats per waypoint = 36 registers):
#   Input double registers:
#     0-5:   waypoint slot 0 (x, y, z, rx, ry, rz)
#     6-11:  waypoint slot 1
#     12-17: waypoint slot 2
#     18-23: waypoint slot 3
#     24-29: waypoint slot 4
#     30-35: waypoint slot 5
#     36:    lin_vel (m/s)
#     37:    acc (m/s²)
#     38:    blend/apo (m)
# NOTE: params use 36-38, so ring_buffer_size must stay <= 6.
#   Input int registers:
#     0: MQ_ACTION (0=idle, 10=LIN queue)
#     1: MQ_CMD_ID
#     2: MQ_PT_CNT (total points)
#     3: MQ_WR_IDX (1-based write cursor, updated by Blender)
#   Output int registers (written by URScript, read by Blender):
#     0: MQ_RD_IDX (1-based read cursor)
#     1: MQ_DONE_ID

RING_BUFFER_SIZE_DEFAULT = 6


def generate_stream_script(ring_buffer_size: int = RING_BUFFER_SIZE_DEFAULT) -> str:
    """Generate URScript that reads waypoints from RTDE input registers
    in a ring buffer loop and executes movel() with blend.

    Direct analogue of KUKA mq_stream.src:
    - KUKA reads MQ_PT[i] via $CONFIG.DAT variables
    - UR reads input_float_register(base + offset) via RTDE
    - Both use rd_idx / wr_idx handshake for ring buffer flow control
    """
    bs = int(max(2, min(6, int(ring_buffer_size or RING_BUFFER_SIZE_DEFAULT))))
    return f"""\
def mq_stream():
  textmsg("mq_stream: starting (ring_size={bs})")

  # Init output registers
  write_output_integer_register(0, 0)
  write_output_integer_register(1, 0)

  local running = True
  local last_cmd_id = -1

  while running:
    local action = read_input_integer_register(0)

    if action == 10:
      # LIN queue mode — read ring buffer and execute movel
      local cmd_id = read_input_integer_register(1)
      local total  = read_input_integer_register(2)
      if cmd_id == last_cmd_id:
        # Already processed this command id; wait for a new stream_start.
        sync()
      elseif total <= 0:
        sync()
      else:
        local rd_idx = 1
        local count  = 0

        local vel   = read_input_float_register(36)
        local acc   = read_input_float_register(37)
        local blend = read_input_float_register(38)

        if vel < 0.001:
          vel = 0.1
        end
        if acc < 0.001:
          acc = 0.5
        end
        if blend < 0.0:
          blend = 0.0
        end

        textmsg("mq_stream: queue start, total=" + to_str(total))

        while count < total:
          local wr_idx = read_input_integer_register(3)

          if rd_idx < wr_idx:
            # Read waypoint from ring buffer slot
            local raw = rd_idx - 1
            local slot = raw - floor(raw / {bs}) * {bs}
            local base = slot * 6

            local px  = read_input_float_register(base + 0)
            local py  = read_input_float_register(base + 1)
            local pz  = read_input_float_register(base + 2)
            local prx = read_input_float_register(base + 3)
            local pry = read_input_float_register(base + 4)
            local prz = read_input_float_register(base + 5)

            # Re-read speed params (allows live update)
            vel   = read_input_float_register(36)
            acc   = read_input_float_register(37)
            blend = read_input_float_register(38)
            if vel < 0.001:
              vel = 0.1
            end
            if acc < 0.001:
              acc = 0.5
            end

            count = count + 1
            rd_idx = rd_idx + 1

            # Update read cursor so Blender knows we consumed this slot
            write_output_integer_register(0, rd_idx)

            # Last point: blend=0 for clean deceleration
            if count >= total:
              movel(p[px, py, pz, prx, pry, prz], a=acc, v=vel, r=0.0)
            elseif blend > 0.0:
              movel(p[px, py, pz, prx, pry, prz], a=acc, v=vel, r=blend)
            else:
              movel(p[px, py, pz, prx, pry, prz], a=acc, v=vel)
            end
          else:
            # Buffer empty — wait for Blender to write more points
            sync()
          end
        end

        # Signal completion
        write_output_integer_register(1, cmd_id)
        write_output_integer_register(0, 0)
        last_cmd_id = cmd_id
        textmsg("mq_stream: queue done, " + to_str(count) + " points")

        sync()
      end

    elseif action == 99:
      # Exit
      running = False

    else:
      # Idle — wait
      sync()
    end
  end

  textmsg("mq_stream: exit")
end
"""
