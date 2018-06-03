import sys

payload_mass = sys.argv[1]
with open('model_template.sdf', 'r') as template:
    data = template.read().replace('\n', '')
    data_filled = data.replace('{PAYLOAD_MASS}', payload_mass)
    with open("model.sdf", "w") as model_file:
        model_file.write(data_filled)
