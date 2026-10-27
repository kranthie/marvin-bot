# Reporting safety and security issues

Marvin Bot is a 40 kg robot on a 48 V battery with joints that produce up to 120 N·m. A design
or software fault can injure people or start a fire, so safety issues are treated like security
vulnerabilities.

Please report privately, not in a public issue:

- anything that could make the robot move unexpectedly, fail to stop on the e-stop, overheat, or
  damage its battery or wiring;
- unsafe values in the description, wiring, bench scripts or policies (torque, current, speed or
  temperature limits);
- security issues in the onboard software or its network interfaces.

Use GitHub's private reporting form for this repository (the **Security** tab, then **Report a
vulnerability**). Describe what you found, where (file and line, or the part and joint), and how
it could cause harm. You will get an answer as soon as possible; fixes and credit are agreed with
you before anything is made public.

This is a hobby project at the design stage (nothing has been built yet), provided as is and
without warranty; reports are handled on a best-effort basis. The safety rules for
working on the robot are in [docs/safety.md](docs/safety.md).
