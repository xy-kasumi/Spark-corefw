## Overview
### State Machine
The FW is a state machine that executes *command*s.
* `IDLE`: machine is not executing command, and ready to accept commands.
* `EXEC_INTERACTIVE`: machine is executing a single isolated command.
* `EXEC_STREAM`: machine is executing a sequence of streamed command.
Allowed transition
* `IDLE` -> `EXEC_INTERACTIVE` | `EXEC_STREAM`
* `EXEC_INTERACTIVE` -> `IDLE`
* `EXEC_STREAM` -> `IDLE`
(i.e. `EXEC_INTERACTIVE` <-> `EXEC_STREAM` is not allowed)

Machine will never execute two commands at the same time.
As shorthand, we refer (`EXEC_INTERACTIVE` or `EXEC_STREAM`) as simply `EXEC`.
### Command Execution
A command is always representable by a single human-readable text.
When the FW enters `EXEC` state, an *execution context* is created.
The context is destroyed when the FW leaves `EXEC` state.

`EXEC_INTERACTIVE` is basically same as single-command `EXEC_STREAM`, and commands should respect that.

In each context, commands are assigned *sequence number* starting from 1 and increments.

Commands can decide to reject to run (cause error) in certain mode (only allow interactive or stream). This is useful for cerain commands that should not be part of a stream.
### Communication
The FW exposes serial communication, as baseline mandatory communication.
Everything FW can do physically (other than pure UI purposes) must be exposed to serial.

Optionally, if the hardware has big storage (e.g. SD card), FW can implement special command that executes EXEC_STREAM based on storage instead of serial streaming. Execution semantics will be exactly the same.
### Error Handling
FW must always maintain defined state and respond to comm.
Only exception is the code that satisfies both:
* is internal pure logic error (no I/O or hardware dependency)
* (is covered by automated tests) OR (no "sane default" to recover to)

This rule does not mean you should check every null-pointer or uncovered switch() that "cannot happen". Instead, this mostly applies to rare but possible things like sudden hardware failure etc. In these cases, ability to keep accessing comm would be important for debugging.

This rule also applies to implementation of cancels, especially `HARD_CANCEL`.
The FW should do cancels with relatively clean code
* OK: calling designated reset() methods etc.
* OK: losing origin or some periphs needs re-init
* NG: stop always running ISRs "just in case"
* NG: program ends up in undefined state
In short, FW should not try to be "as safe as possible" when it potentially increase bugs. Critical safety should be done in electrical or mechanical level if deemed really necessary.

## Serial Protocol

### Stream Syntax
Both of up (FW -> host) & down (host -> FW) streams are line-oriented.
Any of CR or CRLF or LF can be used as the separator.
```
line-raw = {whitespace} line-content [";" comment] {whitespace} line-sep
```

Up & down is full-duplex; the design intentionally allows the host to *not* wait for response, when performance is needed for streaming.

Host & FW should ignore lines with empty `line-content`.
FW can implement backspace support to ease interactive usage.
### Per-Line Syntax
**Input syntax
```
input =
  command |
  ":" nat-number (* seq_num *) " " command |
  "::" |
  "!"

examples:
:1 G1 X1
::
G1 X1
!
```

**Output syntax**
IDLE
```
output-idle =
  "I " human-text
```

EXEC_INTERACTIVE
```
output-interactive =
  ">ack" |
  ">err " human-text |
  ">inf " human-text |
  ">blob " base64-text (* payload in urlsafe base64 w/o "=" *) " " checksum-text (* adler-32 checksum *)

checksum-text =
  [0-9a-f]{8}

examples:
>ack
>err unkown command: Xset
>inf doing processing 1.3
>blob AQIDBA 0018000b
```

EXEC_STREAM
```
output-stream :=
  "@rem " nat-number (* num_currently_acceptable_commands, >=0 *) |
  "@" nat-number (* seq_num *) " err " human-text |
  "@" nat-number (* seq_num *) " inf " human-text |
  "@err " human-text |
  "@inf " human-text

examples:
@rem 100
@34 inf ignoring unknown code
@5 err invalid parameter
@5 err operation failed
@err wrong seq number
@err buffer depleted
@err machine failed suddenly
@inf ignoring smoothing singularity
```

**Syntax Details**
```
nat-number = "0" | "1" | "2" | ...
command = [A-Za-z0-9.+-_ ]+
human-text = [^\n\r]+
```

## Protocol Semantics
Host can
* Any state
	* `SOFT_CANCEL`: Cancel execution by `!`
		* cancels current command's execution and end `EXEC` state.
		* No effect when done in `IDLE`
    * commands are supposed to cancel within 100ms of request.
* In IDLE
	* Start interactive command
	* Start stream commands
		* by issuing `:1 ...`
* In EXEC_INTERACTIVE
	* shoud just wait for completion
* In EXEC_STREAM
	* Feed next command
		* `:...`
	* End stream
		* `::`

All other inputs from host is disallowed and results in error.
When going back to `IDLE` from any of `EXEC` or after `CANCEL`,
at least one `output-idle` must be sent to notify that state is changed.

FW can choose to keep status flag that persists across states, such that
* can be only reset by certain interactive commands
* other commands results in error depending on the flag state
This can be used to ensure initialization, error recovery etc.
### Error Levels
"err" and "inf" is used throughout the protocol.

"err" means `EXEC` cannot continue.
* FW should go back to IDLE. (similar to `SOFT_CANCEL`)
* Host should stop sending further command.

"inf" is everything else (including warnings).


## Commands
```
command =
  "set" setting-name setting-value |
  "get" [setting-name] |
  g-command

g-command =
  ("G" | "M") nat-num ... (* TBD *)


```

* set/get: `EXEC_INTERACRIVE`
* `g-command`: both


Convention
* interactive-only command: starts with lowercase
* both command: starts with uppercase
* stream only command: no such thing?
