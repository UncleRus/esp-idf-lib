# frozen_string_literal: true

class Group
  def initialize(hash)
    raise ArgumentError, "missing key `name`" unless hash.key?("name")

    @metadata = hash
  end

  def description?
    @metadata.key?("description")
  end

  def description
    @metadata["description"]
  end
end
