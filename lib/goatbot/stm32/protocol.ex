defmodule Goatbot.STM32.Protocol do
  @moduledoc """
  Pure encode/decode for the STM32 power-management wire protocol.

  The contract is defined firmware-side in `nsk_stm32/protocol/src/lib.rs`:
  one ASCII message per line at 115200 8N1. The firmware always transmits
  `\\r\\n`; every line it emits begins with `OK`, `ERR`, or `EVT`.

  This module does no I/O. `encode/1` returns a bare line (no terminator)
  as iodata; the transport is responsible for appending `\\r\\n`.
  `decode/1` takes a single line with its terminator already stripped.
  """

  @typedoc "A command to send to the STM32."
  @type command ::
          {:set_rtc, non_neg_integer()}
          | :get_rtc
          | {:sleep, non_neg_integer()}
          | :get_wake_reason
          | :get_power
          | {:set_charge_thresholds, integer(), integer()}
          | {:set_charging, boolean()}
          | :enter_bootloader

  @type wake_reason :: :power_on | :button | :rtc | :unknown

  @type power :: %{
          v_batt_mv: integer(),
          i_batt_ma: integer(),
          usb_present: boolean(),
          chrg_en: boolean()
        }

  @typedoc "A successfully decoded reply, unsolicited event, or error line."
  @type decoded ::
          {:ok,
           :ack_set_rtc
           | {:rtc, non_neg_integer()}
           | :ack_sleep
           | {:wake_reason, wake_reason()}
           | {:power, power()}
           | :ack_set_charge_thresholds
           | :ack_set_charging
           | :ack_enter_bootloader}
          | {:event, {:button, non_neg_integer(), boolean()}}
          | {:error, {String.t(), :badcmd | :badargs | :range | :toolong}}
          | {:unknown, String.t()}

  @doc """
  Encode a command to its bare wire line as iodata (no `\\r\\n`).
  """
  @spec encode(command()) :: iodata()
  def encode({:set_rtc, epoch}), do: ["RTC ", Integer.to_string(epoch)]
  def encode(:get_rtc), do: "RTC?"
  def encode({:sleep, secs}), do: ["SLEEP ", Integer.to_string(secs)]
  def encode(:get_wake_reason), do: "WAKE?"
  def encode(:get_power), do: "PWR?"

  def encode({:set_charge_thresholds, min, max}),
    do: ["CHGLIM ", Integer.to_string(min), " ", Integer.to_string(max)]

  def encode({:set_charging, true}), do: "CHG 1"
  def encode({:set_charging, false}), do: "CHG 0"
  def encode(:enter_bootloader), do: "BOOTLOADER"

  @doc """
  Decode a single received line (terminator already stripped).

  Classifies on the first token, exactly as the firmware guarantees: `OK`,
  `ERR`, or `EVT`. Anything else, or a malformed known line, returns
  `{:unknown, line}` so the caller can log it rather than crash.
  """
  @spec decode(String.t()) :: decoded()
  def decode(line) when is_binary(line) do
    case String.split(line) do
      ["OK" | rest] -> decode_ok(rest, line)
      ["EVT" | rest] -> decode_evt(rest, line)
      ["ERR", verb, reason] -> decode_err(verb, reason, line)
      _ -> {:unknown, line}
    end
  end

  # --- OK replies ---

  defp decode_ok(["RTC"], _line), do: {:ok, :ack_set_rtc}
  defp decode_ok(["SLEEP"], _line), do: {:ok, :ack_sleep}
  defp decode_ok(["CHGLIM"], _line), do: {:ok, :ack_set_charge_thresholds}
  defp decode_ok(["CHG"], _line), do: {:ok, :ack_set_charging}
  defp decode_ok(["BOOTLOADER"], _line), do: {:ok, :ack_enter_bootloader}

  defp decode_ok(["RTC", epoch], line) do
    case parse_int(epoch) do
      {:ok, secs} -> {:ok, {:rtc, secs}}
      :error -> {:unknown, line}
    end
  end

  defp decode_ok(["WAKE", reason], line) do
    case reason do
      "pwr" -> {:ok, {:wake_reason, :power_on}}
      "btn" -> {:ok, {:wake_reason, :button}}
      "rtc" -> {:ok, {:wake_reason, :rtc}}
      "unk" -> {:ok, {:wake_reason, :unknown}}
      _ -> {:unknown, line}
    end
  end

  defp decode_ok(["PWR", vbat, ibat, usb, chrg], line) do
    with {:ok, v} <- parse_int(vbat),
         {:ok, i} <- parse_int(ibat),
         {:ok, usb?} <- parse_bool(usb),
         {:ok, chrg?} <- parse_bool(chrg) do
      {:ok, {:power, %{v_batt_mv: v, i_batt_ma: i, usb_present: usb?, chrg_en: chrg?}}}
    else
      :error -> {:unknown, line}
    end
  end

  defp decode_ok(_rest, line), do: {:unknown, line}

  # --- EVT unsolicited events ---

  defp decode_evt(["BTN", pin, high], line) do
    with {:ok, pin} <- parse_int(pin),
         {:ok, high?} <- parse_bool(high) do
      {:event, {:button, pin, high?}}
    else
      :error -> {:unknown, line}
    end
  end

  defp decode_evt(_rest, line), do: {:unknown, line}

  # --- ERR error lines ---

  defp decode_err(verb, reason, line) do
    case reason do
      "badcmd" -> {:error, {verb, :badcmd}}
      "badargs" -> {:error, {verb, :badargs}}
      "range" -> {:error, {verb, :range}}
      "toolong" -> {:error, {verb, :toolong}}
      _ -> {:unknown, line}
    end
  end

  # --- helpers ---

  defp parse_int(s) do
    case Integer.parse(s) do
      {n, ""} -> {:ok, n}
      _ -> :error
    end
  end

  defp parse_bool("0"), do: {:ok, false}
  defp parse_bool("1"), do: {:ok, true}
  defp parse_bool(_), do: :error
end
