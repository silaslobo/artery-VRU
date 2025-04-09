import xml.etree.ElementTree as ET

# Load the XML file
xml_file = "/home/silaslobo/Development/artery-VRU_branchs/newClusteringContainer/artery-VRU/scenarios/InTAS_VRU/scenario/InTAS_intermodal_updated.rou.xml"  # Replace with your actual XML file path
tree = ET.parse(xml_file)
root = tree.getroot()

# Count the number of 'person' elements
person_count = len(root.findall("person"))
print(f"Number of <person> elements: {person_count}")

person_with_ride_count = sum(1 for person in root.findall("person") if person.find("ride") is not None)
print(f"Number of <person> elements with at least one <ride> child: {person_with_ride_count}")


consecutive_ride_count = 0
for person in root.findall("person"):
    rides = [child for child in person if child.tag == "ride"]  # Get all 'ride' children
    for i in range(len(rides) - 1):
        if rides[i].tag == "ride" and rides[i + 1].tag == "ride":  # Check consecutive rides
            consecutive_ride_count += 1
            break  # Count this person only once, then move to the next

print(f"Number of <person> elements with consecutive <ride> children: {consecutive_ride_count}")

persons_with_repeated_lines = 0

for person in root.findall("person"):
    rides = [child for child in person if child.tag == "ride"]  # Get all 'ride' elements

    if len(rides) > 1:  # Only check if there are at least 2 rides
        seen_lines = set()  # Store seen 'lines' attributes
        for ride in rides:
            lines_attr = ride.get("lines")  # Get the 'lines' attribute
            if lines_attr in seen_lines:  # Check if it's repeated
                persons_with_repeated_lines += 1
                break  # Count this person only once, then move to the next
            seen_lines.add(lines_attr)  # Add to the set

print(f"Number of <person> elements with consecutive <ride> children that repeat a 'lines' value: {persons_with_repeated_lines}")

matching_person_ids = []

for person in root.findall("person"):
    rides = [child for child in person if child.tag == "ride"]  # Get all 'ride' elements

    if len(rides) > 1:  # Only check if there are at least 2 rides
        seen_lines = set()  # Store seen 'lines' attributes
        for ride in rides:
            lines_attr = ride.get("lines")  # Get the 'lines' attribute
            if lines_attr in seen_lines:  # Check if it's repeated
                person_id = person.get("id")  # Get the 'id' of the person
                matching_person_ids.append(person_id)
                break  # Stop checking this person once a match is found
            seen_lines.add(lines_attr)  # Add to the set

# Print all matching person IDs
if matching_person_ids:
    print("Persons with consecutive <ride> elements repeating a 'lines' value:")
    for pid in matching_person_ids:
        print(f"- {pid}")
else:
    print("No persons found with consecutive <ride> elements that repeat a 'lines' value.")