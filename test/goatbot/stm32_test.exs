defmodule Goatbot.STM32Test do
  use ExUnit.Case, async: true

  alias Goatbot.STM32
  alias Goatbot.STM32.FakeUART

  @port "ttyTEST"

  setup do
    {:ok, fake} = FakeUART.start_link(self())

    {:ok, stm} =
      STM32.start_link(
        uart_module: FakeUART,
        uart_pid: fake,
        port: @port,
        name: nil,
        # keep the internal watchdog short so timeout tests are fast
        timeout: 200
      )

    %{stm: stm}
  end

  # Feed a line to the driver exactly as Circuits.UART would in active mode.
  defp feed(stm, line), do: send(stm, {:circuits_uart, @port, line})

  describe "request/response" do
    test "get_power sends PWR? and returns the parsed metrics", %{stm: stm} do
      task = Task.async(fn -> STM32.get_power(stm) end)
      assert_receive {:uart_write, "PWR?"}
      feed(stm, "OK PWR 3812 -142 1 1")

      assert Task.await(task) ==
               {:ok, %{v_batt_mv: 3812, i_batt_ma: -142, usb_present: true, chrg_en: true}}
    end

    test "get_rtc returns the epoch", %{stm: stm} do
      task = Task.async(fn -> STM32.get_rtc(stm) end)
      assert_receive {:uart_write, "RTC?"}
      feed(stm, "OK RTC 1757260800")
      assert Task.await(task) == {:ok, 1_757_260_800}
    end

    test "set_charging sends CHG 1 and returns :ok on ack", %{stm: stm} do
      task = Task.async(fn -> STM32.set_charging(stm, true) end)
      assert_receive {:uart_write, "CHG 1"}
      feed(stm, "OK CHG")
      assert Task.await(task) == :ok
    end

    test "an ERR reply is returned as a structured error", %{stm: stm} do
      task = Task.async(fn -> STM32.set_charging(stm, true) end)
      assert_receive {:uart_write, "CHG 1"}
      feed(stm, "ERR CHG badargs")
      assert Task.await(task) == {:error, {"CHG", :badargs}}
    end

    test "a command times out if no reply arrives", %{stm: stm} do
      assert STM32.get_power(stm) == {:error, :timeout}
    end
  end

  describe "interleaved events" do
    test "an EVT arriving before the reply is broadcast and does not satisfy the command",
         %{stm: stm} do
      :ok = STM32.subscribe(stm)

      task = Task.async(fn -> STM32.get_power(stm) end)
      assert_receive {:uart_write, "PWR?"}

      # Unsolicited button event lands between the command and its reply.
      feed(stm, "EVT BTN 0 1")
      assert_receive {:nsk_stm32_event, {:button, 0, true}}

      # The command must still be pending and completed by the real reply.
      feed(stm, "OK PWR 0 0 0 0")

      assert Task.await(task) ==
               {:ok, %{v_batt_mv: 0, i_batt_ma: 0, usb_present: false, chrg_en: false}}
    end

    test "events are delivered to subscribers outside of any command", %{stm: stm} do
      :ok = STM32.subscribe(stm)
      feed(stm, "EVT BTN 2 0")
      assert_receive {:nsk_stm32_event, {:button, 2, false}}
    end
  end

  describe "client-side validation" do
    test "set_rtc rejects epochs outside the RTC's representable window without writing",
         %{stm: stm} do
      assert STM32.set_rtc(stm, 0) == {:error, :out_of_range}
      assert STM32.set_rtc(stm, 946_684_799) == {:error, :out_of_range}
      assert STM32.set_rtc(stm, 4_102_444_800) == {:error, :out_of_range}
      refute_receive {:uart_write, _}
    end

    test "set_charge_thresholds rejects an inverted pair without writing", %{stm: stm} do
      assert STM32.set_charge_thresholds(stm, 3900, 3500) == {:error, :out_of_range}
      refute_receive {:uart_write, _}
    end

    test "sleep rejects values outside the u32 range without writing", %{stm: stm} do
      assert STM32.sleep(stm, -1) == {:error, :out_of_range}
      assert STM32.sleep(stm, 4_294_967_296) == {:error, :out_of_range}
      refute_receive {:uart_write, _}
    end
  end
end
