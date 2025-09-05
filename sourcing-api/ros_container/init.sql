    -- Table to track if camera features have been initialized
    CREATE TABLE IF NOT EXISTS camera_initialized (
        initialized BOOLEAN PRIMARY KEY
    );

    -- Table for cameras
    CREATE TABLE IF NOT EXISTS cameras (
        id SERIAL PRIMARY KEY,
        identifier VARCHAR(255) UNIQUE NOT NULL,
        type VARCHAR(50),
        config JSONB, -- Store configuration as JSON
        user_notes TEXT,
        publishing_preset VARCHAR(255)
    );

    -- Table for feature groups
    CREATE TABLE IF NOT EXISTS feature_groups (
        id SERIAL PRIMARY KEY,
        name VARCHAR(255) UNIQUE NOT NULL
    );

    -- Table for individual features
    CREATE TABLE IF NOT EXISTS features (
        id SERIAL PRIMARY KEY,
        group_id INTEGER REFERENCES feature_groups(id),
        name VARCHAR(255) NOT NULL,
        type VARCHAR(50),
        value TEXT,
        tooltip TEXT,
        description TEXT,
        min VARCHAR(50),
        max VARCHAR(50),
        options TEXT, -- Store as JSON array string
        representation VARCHAR(50),
        is_writable BOOLEAN DEFAULT FALSE,
        UNIQUE (group_id, name)
    );

    -- Table for storing camera presets
    CREATE TABLE IF NOT EXISTS presets (
        id SERIAL PRIMARY KEY,
        device_identifier VARCHAR(255) NOT NULL, -- Identifier of the device this preset is for
        name VARCHAR(255) NOT NULL, -- Name of the preset
        configuration JSONB, -- Stores the preset configuration in JSON format
        UNIQUE (device_identifier, name) -- Ensure unique preset names per device
    );
