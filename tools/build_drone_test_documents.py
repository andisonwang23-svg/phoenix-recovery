from pathlib import Path
from datetime import date

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.style import WD_STYLE_TYPE
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path("/Users/antonywang/phoenix-recovery 2")
OUT = ROOT / "docs"
OUT.mkdir(parents=True, exist_ok=True)

NAVY = "173A5E"
PALE_BLUE = "EAF2F8"
PALE_GRAY = "F4F6F7"
LIGHT_GRAY = "D9D9D9"
MID_GRAY = "5E6A71"
BLACK = RGBColor(0, 0, 0)


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=100, start=110, bottom=100, end=110):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for name, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{name}"))
        if node is None:
            node = OxmlElement(f"w:{name}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_table_borders(table, color=LIGHT_GRAY, size="6"):
    tbl_pr = table._tbl.tblPr
    borders = tbl_pr.first_child_found_in("w:tblBorders")
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        element = borders.find(qn(f"w:{edge}"))
        if element is None:
            element = OxmlElement(f"w:{edge}")
            borders.append(element)
        element.set(qn("w:val"), "single")
        element.set(qn("w:sz"), size)
        element.set(qn("w:space"), "0")
        element.set(qn("w:color"), color)


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def prevent_row_split(row):
    tr_pr = row._tr.get_or_add_trPr()
    cant_split = OxmlElement("w:cantSplit")
    cant_split.set(qn("w:val"), "true")
    tr_pr.append(cant_split)


def set_col_widths(table, widths):
    for row in table.rows:
        for idx, width in enumerate(widths):
            if idx < len(row.cells):
                row.cells[idx].width = Inches(width)


def style_table(table, widths=None, center_columns=()):
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    set_table_borders(table)
    if widths:
        set_col_widths(table, widths)
    for row_idx, row in enumerate(table.rows):
        prevent_row_split(row)
        if row_idx == 0:
            set_repeat_table_header(row)
        for col_idx, cell in enumerate(row.cells):
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            if row_idx == 0:
                set_cell_shading(cell, NAVY)
            elif row_idx % 2 == 0:
                set_cell_shading(cell, PALE_BLUE)
            else:
                set_cell_shading(cell, "FFFFFF")
            for paragraph in cell.paragraphs:
                paragraph.paragraph_format.space_after = Pt(0)
                paragraph.paragraph_format.space_before = Pt(0)
                paragraph.paragraph_format.line_spacing = 1.05
                if col_idx in center_columns:
                    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
                for run in paragraph.runs:
                    run.font.name = "Arial"
                    run.font.size = Pt(9.2)
                    if row_idx == 0:
                        run.font.bold = True
                        run.font.color.rgb = RGBColor(255, 255, 255)


def add_table(doc, headers, rows, widths=None, center_columns=()):
    table = doc.add_table(rows=1, cols=len(headers))
    for idx, text in enumerate(headers):
        table.rows[0].cells[idx].text = str(text)
    for row_data in rows:
        cells = table.add_row().cells
        for idx, value in enumerate(row_data):
            cells[idx].text = str(value)
    style_table(table, widths=widths, center_columns=center_columns)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return table


def add_hyperlink(paragraph, text, url):
    part = paragraph.part
    rel_id = part.relate_to(url, "http://schemas.openxmlformats.org/officeDocument/2006/relationships/hyperlink", is_external=True)
    hyperlink = OxmlElement("w:hyperlink")
    hyperlink.set(qn("r:id"), rel_id)
    run = OxmlElement("w:r")
    props = OxmlElement("w:rPr")
    color = OxmlElement("w:color")
    color.set(qn("w:val"), "1F5A91")
    underline = OxmlElement("w:u")
    underline.set(qn("w:val"), "single")
    props.append(color)
    props.append(underline)
    run.append(props)
    text_node = OxmlElement("w:t")
    text_node.text = text
    run.append(text_node)
    hyperlink.append(run)
    paragraph._p.append(hyperlink)


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = paragraph.add_run("Page ")
    run.font.size = Pt(8.5)
    fld_char1 = OxmlElement("w:fldChar")
    fld_char1.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = "PAGE"
    fld_char2 = OxmlElement("w:fldChar")
    fld_char2.set(qn("w:fldCharType"), "end")
    run._r.append(fld_char1)
    run._r.append(instr)
    run._r.append(fld_char2)


def configure_document(doc, short_title):
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(0.72)
    section.bottom_margin = Inches(0.66)
    section.left_margin = Inches(0.78)
    section.right_margin = Inches(0.78)

    normal = doc.styles["Normal"]
    normal.font.name = "Arial"
    normal.font.size = Pt(10.7)
    normal.font.color.rgb = BLACK
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.13

    title = doc.styles["Title"]
    title.font.name = "Arial"
    title.font.size = Pt(25)
    title.font.bold = True
    title.font.color.rgb = BLACK
    title.paragraph_format.space_after = Pt(12)

    for style_name, size, before, after in (
        ("Heading 1", 16, 12, 6),
        ("Heading 2", 13, 9, 4),
        ("Heading 3", 11.3, 7, 3),
    ):
        style = doc.styles[style_name]
        style.font.name = "Arial"
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = BLACK
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True

    for style_name in ("List Bullet", "List Number"):
        style = doc.styles[style_name]
        style.font.name = "Arial"
        style.font.size = Pt(10.5)
        style.paragraph_format.space_after = Pt(3)
        style.paragraph_format.line_spacing = 1.08

    source_style = doc.styles.add_style("Source Text", WD_STYLE_TYPE.PARAGRAPH)
    source_style.base_style = doc.styles["Normal"]
    source_style.font.name = "Arial"
    source_style.font.size = Pt(9.3)
    source_style.font.color.rgb = BLACK
    source_style.paragraph_format.space_after = Pt(0)
    source_style.paragraph_format.line_spacing = 0.98

    header = section.header.paragraphs[0]
    header.text = short_title
    header.alignment = WD_ALIGN_PARAGRAPH.LEFT
    for run in header.runs:
        run.font.name = "Arial"
        run.font.size = Pt(8)
        run.font.color.rgb = RGBColor(90, 100, 110)

    footer = section.footer.paragraphs[0]
    add_page_number(footer)


def add_title_page(doc, title, subtitle, purpose):
    paragraph = doc.add_paragraph(style="Title")
    paragraph.add_run(title)
    subtitle_p = doc.add_paragraph()
    subtitle_p.alignment = WD_ALIGN_PARAGRAPH.LEFT
    run = subtitle_p.add_run(subtitle)
    run.bold = True
    run.font.name = "Arial"
    run.font.size = Pt(13)
    run.font.color.rgb = RGBColor(55, 70, 82)
    subtitle_p.paragraph_format.space_after = Pt(20)

    p = doc.add_paragraph(purpose)
    p.paragraph_format.space_after = Pt(18)

    meta = add_table(
        doc,
        ["Document field", "Value"],
        [
            ("Project", "PHOENIX Recovery"),
            ("Document status", "Engineering test planning document"),
            ("Version", "1.0"),
            ("Prepared", date.today().isoformat()),
            ("Primary rule", "Propellers remain removed until every ground test passes"),
        ],
        widths=[1.8, 4.9],
    )
    doc.add_paragraph(
        "This document does not approve a flight. An experienced adult pilot must verify the aircraft, operating site, airspace, external load, battery system, and emergency procedures before any powered test."
    )
    doc.add_page_break()


def add_bullets(doc, items, level=0):
    for item in items:
        p = doc.add_paragraph(style="List Bullet" if level == 0 else "List Bullet 2")
        p.add_run(item)


def add_numbers(doc, items):
    for item in items:
        p = doc.add_paragraph(style="List Number")
        p.add_run(item)


def add_source(doc, number, title, url, note):
    p = doc.add_paragraph(style="Source Text")
    p.paragraph_format.left_indent = Inches(0.18)
    p.paragraph_format.first_line_indent = Inches(-0.18)
    p.add_run(f"[{number}] ").bold = True
    add_hyperlink(p, title, url)
    p.add_run(f". {note}")


def build_test_plan():
    doc = Document()
    configure_document(doc, "PHOENIX Servo Response and Tilt Measurement Drone Test Plan")
    add_title_page(
        doc,
        "PHOENIX Servo Response and Tilt Measurement Drone Test Plan",
        "Controlled validation of servo motion electrical reliability and attitude response",
        "This plan defines a progressive experiment for a homemade quadcopter carrying the PHOENIX servo test equipment. The drone supplies lift and stable positioning. The PHOENIX servos operate an isolated test mechanism and do not control the drone motors. The experiment measures the full chain from command to pulse width, horn motion, line movement, payload attitude response, and safe return to neutral.",
    )

    doc.add_heading("Test objective and main conclusion", level=1)
    doc.add_paragraph(
        "The immediate objective is to determine whether each servo produces repeatable mechanical movement under controlled load without destabilizing the drone or causing a power failure. The experiment can measure command latency, horn angle, line travel, voltage drop, current, payload roll and pitch, angular rate, overshoot, and return to neutral. It cannot determine the true steering authority of a parafoil because an unloaded servo or suspended test mechanism does not reproduce an inflated canopy moving through air."
    )
    doc.add_paragraph(
        "A quadcopter changes attitude by varying motor thrust. A test servo should therefore be mechanically isolated from the drone flight controls. Any drone tilt caused by the test servo is a disturbance to investigate, not evidence of parafoil turn performance."
    )

    doc.add_heading("Scope", level=1)
    add_bullets(doc, [
        "Calibrate Servo 1 and Servo 2 with propellers removed.",
        "Measure actual horn angle and line travel instead of estimating position from command percentage.",
        "Repeat the measurement with a controlled spring or force gauge.",
        "Verify power integrity while motors are inactive and while motors run on a restraint fixture.",
        "Conduct neutral only and incremental low hover tests only after all ground stages pass.",
        "Record synchronized sensor, electrical, command, and video data for every attempt.",
    ])

    doc.add_heading("What the test can establish", level=2)
    add_table(doc, ["Question", "Can this test answer it", "Required evidence"], [
        ("Does each servo move in the commanded direction", "Yes", "Video, horn angle, and line displacement"),
        ("Does the servo return to neutral", "Yes", "Neutral error after each command"),
        ("Does the supply remain stable", "Yes", "Voltage and current recorded during movement"),
        ("Does servo activity disturb the drone", "Yes", "Drone and payload IMU data"),
        ("How fast does the actuator respond", "Yes", "Synchronized command and motion timestamps"),
        ("How fast will the parafoil turn", "No", "Inflated canopy flight data is required"),
        ("What brake command gives the best landing accuracy", "No", "Progressive parafoil testing is required"),
    ], widths=[2.35, 1.35, 3.0], center_columns=(1,))

    doc.add_heading("Roles and authority", level=1)
    add_table(doc, ["Role", "Responsibility", "Authority"], [
        ("Pilot in command", "Operates the drone and monitors airspace and aircraft control", "May terminate and land at any time"),
        ("Test director", "Calls each test point and verifies readiness", "May stop a test before or during a command"),
        ("Safety observer", "Watches people, lines, propellers, battery, and operating boundary", "Calls immediate stop for any hazard"),
        ("Data operator", "Starts logging, identifies test numbers, and confirms files are saved", "Does not distract the pilot during flight"),
    ], widths=[1.25, 3.4, 2.05])
    doc.add_paragraph("One person may perform more than one ground role, but the pilot should not operate the dashboard or inspect data during powered flight.")

    doc.add_heading("System configuration", level=1)
    doc.add_paragraph(
        "Use an inert test article. Do not install a rocket motor, ejection charge, igniter, deployment mechanism, or sharp component. The servo test assembly should reproduce the intended servo orientation, horn geometry, supply voltage, wiring, controller, and approximate mass distribution without introducing launch hardware."
    )
    add_table(doc, ["Subsystem", "Required configuration", "Reason"], [
        ("Drone", "Four motor platform with a dedicated flight controller and manual pilot override", "Keeps motor stabilization separate from the test computer"),
        ("Servo test unit", "Two BW90S servos mounted symmetrically below the propeller plane", "Reduces asymmetric mass and protects the propellers"),
        ("Servo load", "Short line connected to a measured spring or force gauge", "Creates repeatable resistance without loose hanging weights"),
        ("Attitude sensor", "BNO085 rigidly mounted with marked axes", "Provides orientation and angular velocity"),
        ("Altitude sensor", "BMP388 with a protected pressure opening", "Records altitude and pressure but does not measure tilt"),
        ("Video", "Camera view of servo marks and surrounding frame", "Provides independent timing and motion evidence"),
        ("Power", "Separate regulated servo supply with common signal ground", "Prevents servo current from passing through the test computer"),
        ("Emergency behavior", "Loss of command requests neutral and does not affect motor stabilization", "Keeps failure response deterministic"),
    ], widths=[1.25, 3.65, 1.8])

    doc.add_heading("Coordinate system", level=2)
    doc.add_paragraph(
        "Mark a body fixed coordinate frame on the test article before recording data. Use positive X forward, positive Y to the right, and positive Z downward if matching common aerospace body axes. Record the exact convention in the log header. The sign of roll, pitch, yaw, and servo commands must remain consistent in the firmware, dashboard, analysis sheet, and test report."
    )
    add_table(doc, ["Quantity", "Definition for this plan", "Unit"], [
        ("Servo command", "Requested brake or test movement relative to neutral", "percent and microseconds"),
        ("Horn angle", "Measured angular displacement from neutral", "degrees"),
        ("Line travel", "Measured linear movement from neutral", "millimeters"),
        ("Roll", "Rotation about the forward body axis", "degrees"),
        ("Pitch", "Rotation about the lateral body axis", "degrees"),
        ("Angular rate", "Magnitude and axis components from the IMU", "degrees per second"),
    ], widths=[1.3, 4.45, 1.05], center_columns=(2,))

    doc.add_heading("Safety controls", level=1)
    add_bullets(doc, [
        "Remove all propellers for servo calibration, sensor setup, software development, and power troubleshooting.",
        "Never hold a powered drone by hand and never tether it to a person.",
        "Use a purpose built restraint fixture for motor tests and maintain clearance from the propeller plane.",
        "Keep every test line short, restrained, and physically unable to reach a propeller.",
        "Operate outdoors only over unoccupied ground with visual line of sight and applicable airspace authorization.",
        "Use an experienced adult pilot and follow the aircraft, battery, flight controller, and local operating instructions.",
        "Place the neutral command and landing decision above completion of the test matrix.",
    ])

    doc.add_heading("Immediate abort conditions", level=2)
    add_table(doc, ["Condition", "Automatic response", "Human response"], [
        ("Invalid or stale IMU", "Stop the active command and request neutral", "Land or end the ground test"),
        ("Servo rail outside its validated range", "Request neutral and latch a power fault", "Disconnect power after motion stops"),
        ("Unexpected motor or flight controller behavior", "No servo command may override flight control", "Pilot lands immediately"),
        ("Growing oscillation", "Cancel the sequence and hold neutral", "Wait for settling or land"),
        ("Line approaches a propeller", "No recovery action depends on software", "Cut throttle when safe and inspect the rig"),
        ("Communication loss", "Complete only the neutral action and reject new motion", "Pilot lands; data operator records the event"),
        ("Battery swelling heat odor or damage", "No further commands", "Isolate the area and follow battery safety procedures"),
    ], widths=[2.1, 2.8, 1.9])

    doc.add_heading("Instrumentation and synchronized logging", level=1)
    doc.add_paragraph(
        "Use one monotonic clock on the test computer for all command and sensor timestamps. The camera should show a test identifier or visible LED that changes when the command is accepted. This creates an independent timing reference if the software log and physical movement disagree. The BNO085 supports orientation and angular velocity reports at configurable rates and attaches timestamps to reports [1]."
    )
    add_table(doc, ["Signal", "Minimum record content", "Suggested rate"], [
        ("Servo command", "requested percent, pulse width, channel, acceptance result", "every event"),
        ("Servo output", "requested and last written pulse width", "50 hertz"),
        ("BNO085", "quaternion, roll, pitch, yaw, angular rate, validity", "50 to 100 hertz"),
        ("BMP388", "pressure, temperature, altitude estimate, validity", "20 to 50 hertz"),
        ("Electrical", "servo rail voltage, test computer voltage, current if available", "20 to 50 hertz"),
        ("Flight controller", "armed state, flight mode, attitude, vibration, battery", "10 to 50 hertz"),
        ("Camera", "video with test identifier and synchronized start event", "30 frames per second"),
    ], widths=[1.25, 4.05, 1.5], center_columns=(2,))
    doc.add_paragraph("The suggested rates are starting values, not validated requirements. Record actual sample intervals and missing samples instead of assuming the requested rate was achieved.")

    doc.add_heading("Log schema", level=2)
    add_table(doc, ["Field group", "Fields"], [
        ("Identification", "run identifier, test point, configuration version, operator, UTC start time"),
        ("Command", "command timestamp, servo channel, requested percent, PWM microseconds, command source"),
        ("Mechanism", "measured horn angle, line travel, applied load, saturation state"),
        ("Attitude", "quaternion, roll, pitch, yaw, axis rates, calibration state, data age"),
        ("Pressure", "pressure, temperature, relative altitude, vertical speed, data age"),
        ("Power", "battery voltage, regulator output, servo rail voltage, current, brownout flag"),
        ("Flight state", "disarmed, ground motor test, hover, landing, fault, failsafe code"),
    ], widths=[1.45, 5.35])

    doc.add_heading("Measurement definitions", level=2)
    doc.add_paragraph("Command delay equals the movement detection time minus the command acceptance time. Detect movement only after at least three consecutive samples exceed the measured neutral noise band.")
    doc.add_paragraph("Tilt change equals the maximum attitude excursion after the command minus the mean attitude during the precommand baseline. Report roll and pitch separately.")
    doc.add_paragraph("The small range response ratio equals tilt change divided by measured line travel. Report this value only for the suspended mechanism. Do not label it as parafoil turn authority.")
    doc.add_paragraph("For a simple horn, ideal line travel is approximately horn radius multiplied by horn rotation in radians. Use direct measurement because line angle, flex, backlash, and spool geometry change the result.")

    doc.add_heading("Test phase zero readiness review", level=1)
    add_bullets(doc, [
        "Record the complete mass, center of gravity, battery type, flight controller, firmware, servo model, regulator, horn radius, line material, and sensor mounting orientation.",
        "Confirm that the drone manufacturer or experienced builder has accepted the external load and structural attachments.",
        "Confirm the pilot can disarm and land independently of the Raspberry Pi or PHOENIX controller.",
        "Confirm that every active command has an automatic end time and returns to neutral.",
        "Confirm that a reboot leaves both test servos neutral and cannot arm the drone.",
    ])

    doc.add_heading("Test phase one props off calibration", level=1)
    add_numbers(doc, [
        "Remove all propellers and disconnect the propulsion battery if the servo system uses a separate supply.",
        "Secure the airframe so it cannot move and confirm both servos start at neutral.",
        "Place an angle scale behind each horn and a millimeter scale beside each line.",
        "Record ten seconds of neutral sensor and voltage data.",
        "Command Servo 1 through the approved test points, returning to neutral after every point.",
        "Repeat the sequence for Servo 2 and perform five repetitions at each approved point.",
        "Reverse direction only after the current command has returned to neutral and the mechanism has settled.",
        "Inspect mounting screws, horn splines, wires, connectors, and regulator temperature after every set.",
    ])
    add_table(doc, ["Test point", "Command", "Hold", "Repetitions", "Progression condition"], [
        ("N0", "Neutral", "10 seconds", "3", "Noise and drift recorded"),
        ("S1 05", "Servo 1 at 5 percent", "0.5 to 1 second", "5", "Returns to neutral each time"),
        ("S1 10", "Servo 1 at 10 percent", "0.5 to 1 second", "5", "No mechanical binding"),
        ("S1 20", "Servo 1 at 20 percent", "0.5 to 1 second", "5", "Voltage remains acceptable"),
        ("S1 40", "Servo 1 at 40 percent", "0.5 to 1 second", "5", "Only after a measured safe limit"),
        ("S2 series", "Repeat for Servo 2", "same", "same", "Mirror the approved sequence"),
    ], widths=[1.05, 2.05, 1.35, 1.0, 1.35], center_columns=(0, 2, 3))

    doc.add_heading("Test phase two controlled load", level=1)
    doc.add_paragraph(
        "Connect each servo to a short spring or force gauge. Do not suspend a loose weight that can swing into the aircraft. Establish three load levels that remain within the measured mechanical and electrical limits. Repeat the props off test matrix and record the actual load, response delay, angle loss, return error, rail voltage, and current."
    )
    add_table(doc, ["Finding", "Interpretation", "Action"], [
        ("Angle decreases as load increases", "Torque margin or supply is limited", "Reduce travel or load and inspect voltage"),
        ("Delay increases without voltage drop", "Mechanical friction or controller timing may dominate", "Inspect linkage and timestamp path"),
        ("Voltage falls and both servos twitch", "Servo supply or wiring is inadequate", "Stop and repair the power path"),
        ("Neutral position changes after loading", "Horn, spline, mount, or calibration may be slipping", "Do not fly until repeatable"),
    ], widths=[2.0, 2.75, 2.05])

    doc.add_heading("Test phase three fixed suspension", level=1)
    doc.add_paragraph(
        "Suspend the inert test article from a fixed overhead structure before using a drone. Add a separate backup tether. The suspension geometry may imitate the planned payload harness, but direct line shortening is still not equivalent to a parafoil brake input."
    )
    add_numbers(doc, [
        "Record the neutral hanging attitude for ten seconds.",
        "Displace the article by approximately five degrees and release it with servos neutral.",
        "Repeat at approximately ten and fifteen degrees to establish natural pendulum damping.",
        "Enable the experimental straightening controller with the smallest approved command limit.",
        "Repeat each disturbance five times from both directions.",
        "Stop if the controller increases amplitude, adds rapid reversals, or fails to return to neutral.",
    ])

    doc.add_heading("Test phase four restrained motor interference test", level=1)
    doc.add_paragraph(
        "Install propellers only after the earlier phases pass. Use a purpose built restraint fixture and an experienced adult operator. Begin with servos neutral. Increase motor power only within the fixture and aircraft limits while observing supply voltage, test computer resets, IMU quality, camera recording, and servo stability."
    )
    add_bullets(doc, [
        "Test neutral with motors stopped, at idle, and at the approved restrained power points.",
        "Apply one brief 5 percent servo command only after the neutral motor data remains stable.",
        "Progress to 10 and 20 percent only when electrical and sensor data remain valid.",
        "Do not perform rapid reversals or the 40 percent point during the first motor test.",
        "Stop for any reset, twitch, missing data, regulator heating, or unexplained attitude estimate change.",
    ])

    doc.add_heading("Test phase five neutral only hover", level=1)
    doc.add_paragraph(
        "The first free flight carries the inert test hardware with both servos locked at neutral. The pilot evaluates basic control, center of gravity, vibration, external load behavior, and landing. The data operator starts logging before arming and does not interact with the dashboard during flight."
    )
    add_table(doc, ["Entry condition", "Required evidence"], [
        ("Ground phases passed", "Signed checklist and saved log files"),
        ("External load secured", "Independent attachment inspection and backup retention"),
        ("Pilot override confirmed", "Pi power removed without affecting manual flight control"),
        ("Site accepted", "Unoccupied operating area, visual line of sight, and airspace check"),
        ("Weather accepted", "Wind and visibility within pilot and aircraft limits"),
        ("Battery accepted", "Charged, undamaged, secured, and voltage checked under load"),
    ], widths=[2.25, 4.55])

    doc.add_heading("Test phase six incremental hover commands", level=1)
    add_numbers(doc, [
        "Establish a stable low hover and record at least five seconds of baseline data.",
        "Apply one brief Servo 1 command at 5 percent and automatically return to neutral.",
        "Wait until the pilot confirms that the aircraft and test mechanism have settled.",
        "Repeat five times, then complete the same sequence for Servo 2.",
        "Progress to 10 percent and then 20 percent only if the previous level passes.",
        "Do not test 40 percent until it is physically calibrated and the pilot approves the larger disturbance.",
        "Land immediately after any abort condition or unexplained trend.",
    ])

    doc.add_heading("Flight test matrix", level=2)
    add_table(doc, ["Run", "Servo", "Command", "Repetitions", "Required recovery"], [
        ("H0", "None", "Neutral only", "1 flight", "Stable hover and landing"),
        ("H1", "Servo 1", "5 percent", "5", "Neutral and settled before next command"),
        ("H2", "Servo 2", "5 percent", "5", "Neutral and settled before next command"),
        ("H3", "Servo 1", "10 percent", "5", "No growing attitude disturbance"),
        ("H4", "Servo 2", "10 percent", "5", "No growing attitude disturbance"),
        ("H5", "Either", "20 percent", "5 each", "Only after review of H1 through H4"),
        ("H6", "Either", "40 percent", "deferred", "Requires separate approval and measured safe travel"),
    ], widths=[0.75, 1.1, 1.45, 1.25, 2.25], center_columns=(0, 1, 2, 3))

    doc.add_heading("Data reduction and interpretation", level=1)
    doc.add_paragraph("For every command, compute the following values from the synchronized log and compare them with the camera record.")
    add_table(doc, ["Metric", "Calculation", "Meaning"], [
        ("Response delay", "first confirmed movement time minus command acceptance time", "Controller, electrical, and servo latency"),
        ("Horn response", "maximum measured horn angle minus neutral angle", "Actual actuator travel"),
        ("Line response", "maximum measured line position minus neutral position", "Mechanism output"),
        ("Tilt excursion", "maximum roll or pitch minus precommand mean", "Disturbance observed by the IMU"),
        ("Return error", "attitude or line position after settling minus initial neutral", "Residual bias or mechanical slip"),
        ("Overshoot", "peak beyond the final settled response", "Potential oscillation or excessive gain"),
        ("Repeatability", "spread across five identical trials", "Confidence in the mapping"),
        ("Voltage sag", "precommand voltage minus minimum command voltage", "Power path adequacy"),
    ], widths=[1.35, 3.25, 2.2])

    doc.add_heading("Pass criteria for the servo test platform", level=2)
    add_bullets(doc, [
        "Both servos start and end at the calibrated neutral pulse width.",
        "Each accepted command produces the expected direction without binding or saturation.",
        "No command persists beyond its configured duration.",
        "Communication loss and process restart result in neutral outputs.",
        "The test computer, camera, sensors, flight controller, and servos remain powered without resets.",
        "The pilot retains full independent control of the drone.",
        "Repeated commands produce a stable mapping with documented uncertainty.",
    ])
    doc.add_paragraph("A successful drone test does not authorize automatic parafoil guidance. It only qualifies the test platform and the parts of the command path that were exercised.")

    doc.add_heading("Preflight checklist", level=1)
    add_table(doc, ["Check", "Verification", "Status"], [
        ("Propellers and motor directions", "Correct and undamaged", "□"),
        ("Frame and landing gear", "No cracks or loose fasteners", "□"),
        ("Servo mounts and horns", "Secure with measured limits", "□"),
        ("Lines", "Short, restrained, and clear of propellers", "□"),
        ("Flight battery", "Charged, secured, and undamaged", "□"),
        ("Regulators", "Correct output under load", "□"),
        ("IMU and barometer", "Valid data and correct axis labels", "□"),
        ("Camera and storage", "Recording confirmed and space available", "□"),
        ("Neutral failsafe", "Verified immediately before arming", "□"),
        ("Pilot and observers", "Roles assigned and abort call understood", "□"),
        ("Airspace and site", "Authorized and clear", "□"),
    ], widths=[2.15, 4.0, 0.65], center_columns=(2,))

    doc.add_heading("Post test report", level=1)
    add_bullets(doc, [
        "Configuration version and photographs of the tested hardware.",
        "Completed test matrix with passes, failures, aborts, and excluded runs.",
        "Plots of command, horn movement, line movement, voltage, roll, pitch, and angular rate against time.",
        "Mean, range, and variation for each repeated test point.",
        "Explanation of every reset, invalid sensor interval, saturation, oscillation, and unexpected movement.",
        "Decision to repeat, modify, or progress, with the person responsible for the next review.",
    ])

    doc.add_heading("Regulatory and technical references", level=1)
    doc.add_paragraph("These sources support the architecture, instrumentation approach, and operating limits. Local requirements and the selected aircraft documentation still apply.")
    add_source(doc, 1, "CEVA BNO08X Data Sheet", "https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf", "Defines the BNO085 orientation, acceleration, angular velocity, report timing, and interface capabilities.")
    add_source(doc, 2, "Bosch BMP388 Product Information", "https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/", "Defines the sensor operating range, interfaces, sampling capability, and drone altitude application.")
    add_source(doc, 3, "NASA Guided Parafoil System Overview", "https://ntrs.nasa.gov/api/citations/20070026249/downloads/20070026249.pdf", "Describes instrumentation, simulation validation, guidance hardware, and progressive flight testing for guided parafoil systems.")
    add_source(doc, 4, "FAA Recreational Flyers and Community Based Organizations", "https://www.faa.gov/uas/recreational_flyers", "Summarizes visual line of sight, operating category, airspace, and recreational flight requirements.")
    add_source(doc, 5, "FAA Small Unmanned Aircraft Systems Regulations", "https://www.faa.gov/newsroom/small-unmanned-aircraft-systems-uas-regulations-part-107", "Summarizes Part 107 operating requirements and pilot responsibilities.")

    path = OUT / "PHOENIX_SERVO_RESPONSE_AND_TILT_DRONE_TEST_PLAN.docx"
    doc.save(path)
    return path


def build_pi_guide():
    doc = Document()
    configure_document(doc, "Raspberry Pi Servo Test Drone Build Guide")
    add_title_page(
        doc,
        "Raspberry Pi Servo Test Drone Build Guide",
        "Companion computer camera sensor logging and isolated servo testing",
        "This guide defines a practical architecture for a homemade quadcopter that carries a Raspberry Pi, camera, BNO085 IMU, BMP388 barometer, and two test servos. A dedicated autopilot controls the motors and stabilizes the aircraft. The Raspberry Pi records evidence, reads the experimental sensors, communicates with the autopilot, and commands the isolated servo test mechanism.",
    )

    doc.add_heading("Architecture decision", level=1)
    doc.add_paragraph(
        "Do not use Raspberry Pi Linux GPIO timing as the primary motor flight controller. A general purpose operating system can pause or delay a process, while multirotor stabilization requires deterministic sensor and motor control. Use a dedicated ArduPilot compatible flight controller for the motors, receiver, primary IMUs, barometer, GPS, arming, and flight failsafes. Use the Raspberry Pi as a companion computer connected by MAVLink [1]."
    )
    add_table(doc, ["Layer", "Responsibility", "Failure behavior"], [
        ("Pilot and receiver", "Manual control, arming, mode selection, and landing", "Remains functional without the Raspberry Pi"),
        ("Dedicated flight controller", "Motor stabilization, attitude estimation, primary flight sensors, and motor failsafes", "Ignores failed companion functions and preserves pilot control"),
        ("Raspberry Pi", "Camera, experimental IMU and barometer logging, servo experiment, and MAVLink recording", "Stops servo motion and does not stop motor stabilization"),
        ("PCA9685 servo driver", "Stable pulse generation for the two test servos", "Outputs are disabled or returned to calibrated neutral"),
        ("Separate regulators", "Independent regulated power for flight controller, Pi, and servo rail", "A servo fault should not reset the autopilot"),
    ], widths=[1.35, 3.55, 1.9])

    doc.add_heading("System flow", level=2)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(
        "Pilot receiver  →  Flight controller  →  ESCs and motors\n"
        "                              ↕ MAVLink\n"
        "Camera  →  Raspberry Pi  →  Log files\n"
        "BNO085  →       │\n"
        "BMP388  →       └→ PCA9685 → Servo 1 and Servo 2"
    )
    r.font.name = "Courier New"
    r.font.size = Pt(10.2)
    p.paragraph_format.space_before = Pt(8)
    p.paragraph_format.space_after = Pt(12)

    doc.add_heading("Assumptions and design limits", level=1)
    add_bullets(doc, [
        "The airframe is a four motor quadcopter built and inspected with help from an experienced adult builder and pilot.",
        "The vehicle is a servo test platform, not a rocket carrier and not an automatic return aircraft.",
        "The servos operate a short restrained test mechanism and never control the drone motors.",
        "Exact motors, propellers, ESCs, battery, frame, and regulator ratings must be selected after the final takeoff mass is known.",
        "The Pi Zero 2 W is the reference computer because its small form factor includes a camera connector and GPIO. More processing may require a larger Pi and a new power and mass budget.",
        "The first flight is manual and neutral only. Autonomous companion computer control is outside this build stage.",
    ])

    doc.add_heading("Parts and selection requirements", level=1)
    add_table(doc, ["Item", "Reference choice", "Selection requirement"], [
        ("Companion computer", "Raspberry Pi Zero 2 W with headers", "5 volt supply sized for the Pi and camera; secure microSD storage"),
        ("Flight controller", "Pixhawk class ArduPilot compatible controller", "Four motor outputs, receiver input, power module, UART telemetry, logging, and supported firmware"),
        ("Camera", "Raspberry Pi Camera Module 3", "Correct Zero camera cable, secure mount, unobstructed view"),
        ("Experimental IMU", "BNO085 breakout", "3.3 volt logic compatible, rigid mounting, known I2C address"),
        ("Experimental barometer", "BMP388 breakout", "3.3 volt logic compatible, protected pressure opening, known I2C address"),
        ("Servo driver", "PCA9685 breakout", "3.3 volt I2C logic and separate servo V plus terminal"),
        ("Test servos", "Two BW90S servos", "Measured neutral, safe pulse limits, and current under load"),
        ("Pi regulator", "Dedicated switching regulator", "Stable 5 volt output with current and transient margin"),
        ("Servo regulator", "Dedicated 5 volt regulator", "Sized from measured simultaneous servo current, wiring, and connector limits"),
        ("Flight power module", "Controller compatible power module", "Powers and measures the flight controller battery path"),
        ("Airframe propulsion", "Matched frame, motors, propellers, ESCs, and battery", "Manufacturer test data and verified total thrust and electrical limits"),
        ("Safety equipment", "Propeller guards when compatible, restraint fixture, fire resistant battery area", "Does not interfere with control or airflow"),
    ], widths=[1.25, 2.25, 3.3])
    doc.add_paragraph(
        "The Raspberry Pi Zero 2 W product brief specifies a 65 by 30 millimeter board, a CSI camera connector, GPIO, and 5 volt input power. Raspberry Pi recommends a 5 volt 2.5 amp supply for Zero models [2]. This rating does not include servo power because the servos use their own regulator."
    )

    doc.add_heading("Mass and propulsion selection", level=1)
    doc.add_paragraph(
        "Complete the mass budget before choosing propulsion parts. Include the frame, motors, ESCs, propellers, fasteners, flight controller, receiver, GPS if used, Pi, camera, sensors, regulators, wiring, servos, test mechanism, landing gear, and battery. Weigh the assembled vehicle instead of relying only on catalog masses."
    )
    add_table(doc, ["Calculation", "Method"], [
        ("Total takeoff mass", "Sum measured installed mass for every component and wire"),
        ("Vehicle weight", "Total takeoff mass multiplied by gravitational acceleration"),
        ("Hover thrust per motor", "Vehicle weight divided by four for a balanced quadcopter"),
        ("Available thrust", "Use manufacturer test data for the exact motor, propeller, voltage, and ESC combination"),
        ("Electrical maximum", "Use measured or published full power current multiplied by four plus avionics and margin"),
    ], widths=[2.1, 4.7])
    doc.add_paragraph("Have an experienced builder verify that thrust, battery discharge, ESC rating, propeller clearance, frame stiffness, and center of gravity remain acceptable after the complete test equipment is installed.")

    doc.add_heading("Required tools", level=2)
    add_bullets(doc, [
        "Digital multimeter and a current measurement method rated for the expected circuit.",
        "Soldering tools, heat shrink, strain relief, and insulated connectors.",
        "Servo angle scale, millimeter ruler, spring scale or force gauge, and a camera timing marker.",
        "Propeller removal tool and a purpose built ground restraint fixture.",
        "Computer for Raspberry Pi OS preparation and flight controller configuration.",
        "Scale for total mass and center of gravity checks.",
    ])

    doc.add_heading("Electrical power architecture", level=1)
    doc.add_paragraph(
        "Use separate regulated branches from the flight battery. The flight controller branch powers the autopilot through its approved power module. The Pi branch powers the Raspberry Pi and camera. The servo branch powers the servo rail. Connect signal grounds at a controlled common point so I2C, UART, and servo signals share a reference, but do not route servo current through the Pi or flight controller ground pins."
    )
    add_table(doc, ["Branch", "Loads", "Key rule"], [
        ("Propulsion", "Four ESCs and motors", "Direct battery distribution sized for maximum current"),
        ("Flight controller", "Autopilot, receiver, primary GPS and approved peripherals", "Use the controller power module and its documented ports"),
        ("Pi", "Raspberry Pi, camera, experimental sensors, PCA9685 logic", "Dedicated stable 5 volt regulator; no servo current"),
        ("Servo", "Servo 1 and Servo 2", "Dedicated 5 volt regulator and local bulk capacitance selected from measurement"),
    ], widths=[1.25, 3.0, 2.55])
    doc.add_paragraph("Never connect regulator outputs together unless the manufacturers explicitly support current sharing. If the flight controller and Pi have separate 5 volt sources, connect only the required signal and common ground unless the controller documentation specifies otherwise.")

    doc.add_heading("Power budget worksheet", level=2)
    add_table(doc, ["Load", "Voltage", "Measured normal current", "Measured peak current", "Regulator branch"], [
        ("Raspberry Pi Zero 2 W", "5 volts", "TBD", "TBD", "Pi"),
        ("Camera Module 3", "from Pi", "TBD", "TBD", "Pi"),
        ("BNO085 and BMP388", "per breakout", "TBD", "TBD", "Pi"),
        ("PCA9685 logic", "3.3 volts", "TBD", "TBD", "Pi"),
        ("Servo 1", "5 volts", "TBD", "TBD", "Servo"),
        ("Servo 2", "5 volts", "TBD", "TBD", "Servo"),
    ], widths=[1.7, 1.0, 1.55, 1.55, 1.0], center_columns=(1, 2, 3, 4))
    doc.add_paragraph("Measure peak current with the intended mechanical load. Do not size a regulator from unloaded servo current.")

    doc.add_heading("Brownout verification", level=2)
    add_numbers(doc, [
        "Remove propellers and connect the complete electronics system.",
        "Measure the Pi regulator output at the Pi power input and the servo rail at the servo connector.",
        "Start camera recording and all sensors.",
        "Command one servo, then both servos, through the approved ground test range.",
        "Record minimum voltage and check for Pi undervoltage, flight controller reset, camera interruption, or sensor corruption.",
        "Reject the power design if a servo command resets or disturbs the flight controller or Pi.",
    ])

    doc.add_heading("Raspberry Pi wiring", level=1)
    doc.add_paragraph(
        "The following pin plan uses the primary Raspberry Pi I2C bus for the experimental sensors and PCA9685. Raspberry Pi GPIO uses 3.3 volt logic [3]. Confirm the specific breakout boards accept the selected supply and logic levels before connecting them."
    )
    add_table(doc, ["Pi physical pin", "GPIO or signal", "Connects to", "Notes"], [
        ("1", "3.3 volts", "BNO085, BMP388, PCA9685 logic VCC when supported", "Do not connect to servo V plus"),
        ("3", "GPIO 2 SDA", "BNO085 SDA, BMP388 SDI or SDA, PCA9685 SDA", "Shared I2C data"),
        ("5", "GPIO 3 SCL", "BNO085 SCL, BMP388 SCK or SCL, PCA9685 SCL", "Shared I2C clock"),
        ("6", "Ground", "Sensor and PCA9685 logic ground", "Join to controlled common ground"),
        ("8", "GPIO 14 TX", "Flight controller telemetry RX", "3.3 volt UART; cross TX to RX"),
        ("10", "GPIO 15 RX", "Flight controller telemetry TX", "3.3 volt UART; cross RX to TX"),
        ("Power input", "5 volts", "Dedicated Pi regulator", "Use a secure power connection and short wiring"),
        ("CSI connector", "Camera interface", "Camera Module 3", "Use the correct Pi Zero camera cable"),
    ], widths=[1.05, 1.35, 2.8, 1.6], center_columns=(0,))

    doc.add_heading("I2C address plan", level=2)
    add_table(doc, ["Device", "Typical address", "Verification"], [
        ("PCA9685", "0x40", "Verify board address jumpers and scan result"),
        ("BNO085", "0x4A or 0x4B", "Verify breakout address selection and library"),
        ("BMP388", "0x76 or 0x77", "Verify SDO state and scan result"),
    ], widths=[1.65, 1.65, 3.5], center_columns=(1,))
    doc.add_paragraph("A scan proves that a device acknowledged an address; it does not prove that the correct data, rate, orientation, or calibration is being used.")

    doc.add_heading("Servo driver and servo wiring", level=2)
    add_table(doc, ["Connection", "Destination", "Rule"], [
        ("PCA9685 VCC", "Pi 3.3 volt logic", "Logic only"),
        ("PCA9685 SDA and SCL", "Pi GPIO 2 and GPIO 3", "Keep wires short and secure"),
        ("PCA9685 ground", "Common signal ground", "Required for valid control pulses"),
        ("PCA9685 servo V plus", "Dedicated regulated 5 volt servo supply", "Never draw servo power from the Pi 3.3 volt pin"),
        ("Servo signal", "PCA9685 selected channel", "Servo 1 and Servo 2 use separate channels"),
        ("Servo positive", "Servo V plus rail", "Confirm connector orientation"),
        ("Servo negative", "Servo ground rail", "Use wiring sized for measured current"),
    ], widths=[1.7, 2.65, 2.8])
    doc.add_paragraph("The PCA9685 provides 12 bit programmable PWM outputs over I2C [4]. It does not measure actual servo position. Use video, an encoder, or a separate feedback sensor when measured position is required.")

    doc.add_heading("Sensor and camera installation", level=1)
    doc.add_heading("BNO085 installation", level=2)
    add_bullets(doc, [
        "Mount the board rigidly on the test assembly and mark its axes.",
        "Place it away from high current motor and battery wires when possible.",
        "Record mounting rotation in the configuration file instead of correcting signs later in analysis.",
        "Use the rotation vector or selected fused orientation report consistently and log calibration status.",
        "Record raw or calibrated angular velocity in addition to roll, pitch, and yaw.",
    ])
    doc.add_paragraph("The BNO085 integrates accelerometer, gyroscope, magnetometer, and onboard sensor fusion, providing orientation and calibrated motion outputs [5].")

    doc.add_heading("BMP388 installation", level=2)
    add_bullets(doc, [
        "Keep the pressure opening exposed to ambient static pressure but protected from direct propeller wash, sunlight, dust, and contact.",
        "Do not seal the pressure port inside an airtight enclosure.",
        "Log pressure and temperature and establish relative altitude from a ground baseline.",
        "Use filtering and reject isolated pressure spikes; do not use this experimental sensor as the only flight altitude source.",
    ])
    doc.add_paragraph("The BMP388 supports I2C and SPI, pressure and temperature measurement, configurable oversampling, and an internal IIR filter [6].")

    doc.add_heading("Camera installation", level=2)
    add_numbers(doc, [
        "Power off the Pi before inserting or removing the camera cable.",
        "Use the correct narrow camera cable for the Pi Zero connector and verify contact orientation.",
        "Mount the camera so the frame, test mechanism, or a timing LED is visible without placing the camera near a propeller.",
        "Add strain relief so vibration cannot pull the cable from the CSI connector.",
        "Test autofocus and exposure on the ground and use a recording mode that the storage card can sustain.",
    ])
    doc.add_paragraph("Raspberry Pi documents Camera Module 3 as a 12 megapixel autofocus camera and supports recording through the current rpicam software [7].")

    doc.add_heading("Mechanical placement", level=1)
    add_table(doc, ["Component", "Recommended location", "Reason"], [
        ("Flight controller", "At the airframe center near the center of gravity", "Reduces lever arm and supports valid vibration isolation"),
        ("Raspberry Pi", "Near the center with airflow and protected wiring", "Limits balance change and supports cooling"),
        ("Camera", "Forward or downward with rigid mount", "Provides a clear evidence view"),
        ("Test servos", "Symmetrical below the propeller plane", "Reduces imbalance and line exposure"),
        ("Servo regulator", "Close to servos with short power wiring", "Reduces voltage drop"),
        ("BMP388", "Protected static pressure area", "Reduces propeller wash pressure errors"),
        ("BNO085", "Rigid experimental platform", "Keeps measured motion tied to the test mechanism"),
        ("Antennas", "Clear of carbon, high current wiring, and each other", "Reduces shielding and interference"),
    ], widths=[1.35, 3.15, 2.3])

    doc.add_heading("Assembly sequence", level=1)
    doc.add_heading("Airframe and propulsion assembly", level=2)
    add_numbers(doc, [
        "Assemble the frame and landing structure according to the component instructions.",
        "Mount motors and ESCs without propellers and verify fasteners, wire protection, and motor clearance.",
        "Install the battery tray so the final center of gravity can be adjusted.",
        "Install the power distribution and flight controller power module with correct polarity and insulation.",
        "Continuity check the main power path before connecting a battery.",
    ])

    doc.add_heading("Flight controller installation", level=2)
    add_numbers(doc, [
        "Mount the controller in its documented orientation and near the vehicle center.",
        "Connect the receiver, ESC signal lines, power module, buzzer, safety switch, and GPS if required.",
        "Load supported multirotor firmware and select the correct frame type.",
        "Calibrate the primary accelerometers, compass, receiver, ESC protocol, battery monitor, and flight modes using the official flight controller instructions.",
        "Configure radio loss, battery, geofence, and arming checks before installing propellers.",
    ])

    doc.add_heading("Raspberry Pi preparation", level=2)
    add_numbers(doc, [
        "Install the current Raspberry Pi OS Lite release on a reliable microSD card.",
        "Set a unique host name and strong password and apply system updates while on the bench.",
        "Enable I2C and the serial hardware. Disable the serial login shell if the UART is used for MAVLink.",
        "Install the approved sensor, camera, MAVLink, and logging libraries in a project virtual environment.",
        "Create a read only configuration file containing addresses, axis rotations, neutral pulses, safe limits, rates, and configuration version.",
        "Configure the logger to start only after all output channels have been set to neutral.",
    ])

    doc.add_heading("Camera verification", level=2)
    p = doc.add_paragraph()
    p.add_run("List cameras with ").bold = False
    code = p.add_run("rpicam-hello --list-cameras")
    code.font.name = "Courier New"
    code.font.size = Pt(10)
    p.add_run(". Record a short bench video and verify that the file closes correctly after an orderly shutdown.")
    doc.add_paragraph("Raspberry Pi camera documentation explains auto detection, current rpicam tools, resolution options, and cable troubleshooting [7].")

    doc.add_heading("MAVLink connection", level=2)
    doc.add_paragraph(
        "Connect Pi TX to flight controller RX, Pi RX to flight controller TX, and common ground. Enable the Pi UART and configure the selected autopilot telemetry port for MAVLink 2 at an approved baud rate. ArduPilot documents Raspberry Pi communication through the serial port and MAVLink [8]. Test telemetry while the flight controller is disarmed."
    )

    doc.add_heading("Software organization", level=1)
    doc.add_paragraph("Separate the experiment into small processes so a camera or dashboard failure cannot hold a servo command or interfere with flight control.")
    add_table(doc, ["Process", "Responsibility", "Critical behavior"], [
        ("Supervisor", "Starts services, checks health, and owns the experiment state", "Sets neutral before any other output"),
        ("IMU reader", "Reads quaternion, orientation, rates, calibration, and data age", "Marks stale data rather than repeating it indefinitely"),
        ("Barometer reader", "Reads pressure and temperature and computes a relative estimate", "Rejects invalid and isolated impossible changes"),
        ("MAVLink reader", "Records flight controller state, attitude, battery, mode, and arming", "Never substitutes for the pilot"),
        ("Servo executor", "Applies bounded, time limited commands through PCA9685", "Returns to neutral on timeout or process loss"),
        ("Camera recorder", "Records video and an event marker", "Failure does not change servo or flight state"),
        ("Log writer", "Writes synchronized records and flushes them safely", "Preserves partial data after an abort"),
    ], widths=[1.35, 3.35, 2.1])

    doc.add_heading("Required software states", level=2)
    add_table(doc, ["State", "Servo behavior", "Allowed transition"], [
        ("Boot", "Neutral requested before sensor initialization", "To bench safe after self test"),
        ("Bench safe", "Manual bounded tests allowed with motors disarmed", "To flight neutral only after readiness review"),
        ("Flight neutral only", "Both servos remain neutral", "To incremental test only by explicit pilot approved action"),
        ("Incremental test", "One time limited command at a time", "Back to neutral after every command"),
        ("Fault", "Neutral command latched", "Requires landing or disarmed reset"),
        ("Shutdown", "Neutral written before services stop", "Power removal after outputs are safe"),
    ], widths=[1.35, 3.45, 2.0])

    doc.add_heading("Servo configuration fields", level=2)
    add_table(doc, ["Field", "Purpose", "Initial status"], [
        ("servo_1_neutral_us", "Measured neutral pulse", "Must be calibrated"),
        ("servo_2_neutral_us", "Measured neutral pulse", "Must be calibrated"),
        ("servo_1_min_us and max_us", "Measured safe travel limits", "Ground test only"),
        ("servo_2_min_us and max_us", "Measured safe travel limits", "Ground test only"),
        ("maximum_test_percent", "Caps all dashboard and automated commands", "Begin at 5 percent"),
        ("command_timeout_ms", "Forces automatic return to neutral", "Must be finite"),
        ("minimum_reversal_delay_ms", "Prevents rapid direction changes", "Requires test validation"),
        ("configuration_version", "Identifies the setup used for each log", "Required"),
    ], widths=[2.25, 3.25, 1.3])

    doc.add_heading("Logging directory", level=2)
    add_table(doc, ["File", "Content"], [
        ("run metadata json", "configuration, software version, operators, hardware, calibration, and test notes"),
        ("telemetry csv", "time aligned sensor, command, flight controller, and power values"),
        ("events csv", "state changes, command acceptance, neutral return, faults, and operator marks"),
        ("video h264 or mp4", "camera record with run identifier"),
        ("summary json", "computed delay, movement, voltage, attitude, and pass results"),
    ], widths=[2.0, 4.8])

    doc.add_heading("Bring up and verification plan", level=1)
    add_table(doc, ["Stage", "Propellers", "Goal", "Exit evidence"], [
        ("One", "Removed", "Verify each regulator and polarity", "Measured voltages recorded"),
        ("Two", "Removed", "Boot flight controller and confirm manual receiver", "Arming remains inhibited during setup"),
        ("Three", "Removed", "Boot Pi, camera, I2C sensors, and logger", "Complete timestamped log"),
        ("Four", "Removed", "Calibrate servo neutral and safe travel", "Five repeatable cycles per point"),
        ("Five", "Removed", "Run simultaneous camera, sensors, telemetry, and servos", "No resets or stale data"),
        ("Six", "Installed in fixture", "Check motor vibration and electrical interference", "No reset, twitch, or data corruption"),
        ("Seven", "Installed", "Neutral only low hover", "Stable manual flight and landing"),
        ("Eight", "Installed", "Incremental 5 percent servo experiment", "Safe neutral recovery and complete data"),
    ], widths=[0.7, 1.1, 3.0, 2.0], center_columns=(0, 1))

    doc.add_heading("Ground acceptance tests", level=2)
    add_bullets(doc, [
        "Power polarity and insulation inspection.",
        "Flight controller remains operational if the Pi is turned off or restarted.",
        "Flight controller remains operational if the servo regulator is disconnected.",
        "Servo command cannot exceed configured pulse and percentage limits.",
        "Killing the experiment process causes neutral outputs.",
        "Unplugging the IMU or barometer produces a logged fault and neutral output.",
        "Camera failure does not affect servo or flight control state.",
        "Five minute and full planned duration tests show no delayed servo shutdown or software timeout.",
        "Simultaneous servo movement does not produce a Pi undervoltage warning or flight controller reset.",
    ])

    doc.add_heading("First flight procedure", level=2)
    add_numbers(doc, [
        "Review weather, operating area, airspace, roles, and abort calls.",
        "Power the aircraft with propellers clear and wait for flight controller and Pi readiness.",
        "Confirm the servo state is neutral and the experiment mode is neutral only.",
        "Start logging and camera recording before arming.",
        "Perform a short manual takeoff and low hover with no servo movement.",
        "Land, disarm, stop the recording, and inspect the logs before any active servo flight.",
    ])

    doc.add_heading("Failure responses", level=1)
    add_table(doc, ["Failure", "Servo response", "Flight response"], [
        ("Pi process crash", "Hardware or supervisor timeout requests neutral", "Dedicated autopilot and pilot continue control"),
        ("Pi power loss", "Servo output must not remain at an aggressive command", "Pilot lands"),
        ("PCA9685 communication loss", "Supervisor enters fault and disables further testing", "Pilot lands"),
        ("BNO085 invalid", "Cancel active test and neutralize", "Pilot lands or ends the run"),
        ("BMP388 invalid", "Log fault; tilt test remains prohibited until review", "Primary autopilot barometer remains responsible for flight"),
        ("Camera failure", "No automatic servo change", "Finish only a neutral safe segment and land"),
        ("Servo supply sag", "Neutralize and latch power fault", "Pilot lands"),
        ("MAVLink loss", "Cancel the experiment and neutralize", "Pilot controls the drone directly"),
        ("Radio control loss", "Companion computer does not invent a recovery command", "Autopilot executes its configured radio failsafe"),
    ], widths=[1.55, 2.75, 2.5])

    doc.add_heading("Build completion checklist", level=1)
    add_table(doc, ["Requirement", "Evidence", "Complete"], [
        ("Final takeoff mass and center of gravity measured", "Mass sheet and balance photograph", "□"),
        ("Propulsion combination supported by test data", "Motor and propeller data sheet", "□"),
        ("Flight controller calibrated and arming checks active", "Configuration backup", "□"),
        ("Manual pilot control independent of Pi", "Power isolation test", "□"),
        ("Pi 5 volt rail validated under maximum planned load", "Voltage log", "□"),
        ("Servo rail validated under simultaneous load", "Voltage and current log", "□"),
        ("BNO085 axes and calibration verified", "Static orientation test", "□"),
        ("BMP388 pressure response verified", "Ground pressure log", "□"),
        ("Camera records and closes files correctly", "Playback test", "□"),
        ("Servo pulse limits measured", "Calibration record", "□"),
        ("Neutral on restart timeout and data loss verified", "Fault injection results", "□"),
        ("All wiring restrained from propellers", "Independent inspection", "□"),
        ("Neutral only flight completed", "Flight log and review", "□"),
    ], widths=[3.25, 2.9, 0.65], center_columns=(2,))

    doc.add_heading("Information still required for a final hardware bill", level=1)
    doc.add_paragraph("The following values are necessary before selecting exact motors, propellers, ESCs, battery, frame, and regulators.")
    add_table(doc, ["Required value", "Why it matters"], [
        ("Target total takeoff mass", "Determines thrust and battery energy"),
        ("Required flight time", "Determines battery capacity and thermal load"),
        ("Maximum frame diameter", "Limits propeller size and component placement"),
        ("Desired camera direction and resolution", "Determines mount, storage, and processing load"),
        ("Exact BW90S voltage and measured current", "Determines servo regulator and wiring"),
        ("Maximum test line load", "Determines servo travel, linkage, and structural attachment"),
        ("Pilot radio and preferred flight controller", "Determines receiver, ports, and setup procedure"),
    ], widths=[2.65, 4.15])

    doc.add_heading("Technical references", level=1)
    add_source(doc, 1, "ArduPilot Companion Computers", "https://ardupilot.org/dev/docs/companion-computers.html", "Defines the companion computer role and MAVLink connection to an autopilot.")
    add_source(doc, 2, "Raspberry Pi Zero 2 W Product Brief", "https://pip-assets.raspberrypi.com/categories/584-raspberry-pi-zero-2-w/documents/RP-008359-DS/raspberry-pi-zero-2-w-product-brief.pdf", "Provides dimensions, interfaces, camera connector, and input power information.")
    add_source(doc, 3, "Raspberry Pi Computer Hardware Documentation", "https://www.raspberrypi.com/documentation/computers/raspberry-pi.html", "Documents 3.3 volt GPIO, bus functions, and recommended power supplies.")
    add_source(doc, 4, "NXP PCA9685 Data Sheet", "https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf", "Defines the I2C controlled 12 bit PWM generator used for isolated servo pulse output.")
    add_source(doc, 5, "CEVA BNO08X Data Sheet", "https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf", "Defines BNO085 sensing, fusion outputs, report timing, and interfaces.")
    add_source(doc, 6, "Bosch BMP388 Data Sheet", "https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf", "Defines BMP388 electrical, interface, filtering, pressure, and temperature behavior.")
    add_source(doc, 7, "Raspberry Pi Camera Documentation", "https://www.raspberrypi.com/documentation/computers/camera_software.html", "Explains supported camera modules, rpicam tools, recording options, and troubleshooting.")
    add_source(doc, 8, "ArduPilot Raspberry Pi MAVLink Connection", "https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html", "Explains Pi UART setup and MAVLink communication with a flight controller.")
    add_source(doc, 9, "FAA Recreational Flyers and Community Based Organizations", "https://www.faa.gov/uas/recreational_flyers", "Summarizes visual line of sight and recreational operating requirements.")

    path = OUT / "RASPBERRY_PI_SERVO_TEST_DRONE_BUILD_GUIDE.docx"
    doc.save(path)
    return path


if __name__ == "__main__":
    outputs = [build_test_plan(), build_pi_guide()]
    for output in outputs:
        print(output)
